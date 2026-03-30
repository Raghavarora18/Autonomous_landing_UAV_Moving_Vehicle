import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridg

import cv2
import cv2.aruco as aruco
import numpy as np

# CONFIGURATION
FOV_RAD     = 1.3962634   
IMAGE_W     = 640
IMAGE_H     = 480
MARKER_SIZE = 1.0      
EXPECTED_ID = 0            

# Coasting: keep publishing Kalman prediction this many frames after loss
MAX_COAST_FRAMES = 15

# Landing confirmation thresholds
LAND_CONFIRM_ALT_M  = 0.8   
LAND_CONFIRM_ERR_M  = 0.4  

# Speed display: smooth km/h reading with EMA
SPEED_EMA_ALPHA = 0.3 

_fx = (IMAGE_W / 2.0) / math.tan(FOV_RAD / 2.0)
_fy = _fx
_cx = IMAGE_W  / 2.0
_cy = IMAGE_H  / 2.0

class MarkerKalmanTracker:
    def __init__(self, x0: float, y0: float):
        self.H = np.array([[1,0,0,0],[0,1,0,0]], dtype=np.float64)
        self.Q = np.diag([0.05, 0.05, 1.0, 1.0]).astype(np.float64)
        self.R = np.diag([0.02, 0.02]).astype(np.float64)
        self.x = np.array([x0, y0, 0.0, 0.0], dtype=np.float64)
        self.P = np.eye(4, dtype=np.float64) * 0.5

    def _F(self, dt):
        return np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]],
                        dtype=np.float64)

    def predict(self, dt):
        F = self._F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        return self.x[:2].copy()

    def update(self, z):
        y  = z - self.H @ self.x
        S  = self.H @ self.P @ self.H.T + self.R
        K  = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        return self.x[:2].copy()

    @property
    def position(self): return self.x[:2].copy()
    @property
    def velocity(self): return self.x[2:4].copy()

def _make_params():
    p = aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMin  = 3
    p.adaptiveThreshWinSizeMax  = 23
    p.adaptiveThreshWinSizeStep = 4
    p.adaptiveThreshConstant    = 7
    p.minMarkerPerimeterRate    = 0.05
    p.maxMarkerPerimeterRate    = 4.0
    p.polygonalApproxAccuracyRate = 0.04
    p.errorCorrectionRate       = 0.6
    p.cornerRefinementMethod    = aruco.CORNER_REFINE_NONE
    return p

class ArucoNode(Node):

    def __init__(self):
        super().__init__('aruco_node')

        self.bridge   = CvBridge()
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_pose', 10)
        self.sub      = self.create_subscription(
            Image, '/drone/downward_camera/image_raw',
            self.image_callback, 10)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.params     = _make_params()

        self.K = np.array([[_fx,0,_cx],[0,_fy,_cy],[0,0,1]], dtype=np.float64)
        self.D = np.zeros((5,1), dtype=np.float64)

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

        self.tracker:        MarkerKalmanTracker | None = None
        self.coast_cnt       = 0
        self.last_z          = 8.0
        self.prev_t          = None
        self.total_frames    = 0
        self.detected_frames = 0

        # Speed display state
        self.speed_kmh_smooth = 0.0   # EMA-smoothed speed in km/h

        self.get_logger().info(
            f"ArUco node ready | fx={_fx:.1f}px | "
            f"marker={MARKER_SIZE}m | id={EXPECTED_ID} | "
            f"coast={MAX_COAST_FRAMES}fr"
        )

    
    def image_callback(self, msg: Image):
        t  = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = float(np.clip(t - self.prev_t, 0.005, 0.2)) if self.prev_t else 0.033
        self.prev_t = t

        frame     = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray      = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        processed = self.clahe.apply(gray)
        self.total_frames += 1

        # Detection 
        corners, ids, _ = aruco.detectMarkers(
            processed, self.aruco_dict, parameters=self.params)

        valid_idx = None
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                if EXPECTED_ID is None:
                    self.get_logger().info(
                        f"Detected ID={mid} — set EXPECTED_ID={mid} to lock on")
                    valid_idx = i; break
                elif mid == EXPECTED_ID:
                    valid_idx = i; break

        detected = valid_idx is not None
        tvecs = None

        if detected:
            try:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    [corners[valid_idx]], MARKER_SIZE, self.K, self.D)
            except Exception as e:
                self.get_logger().warn(f"Pose est failed: {e}")
                detected = False

        if detected and tvecs is not None:
            self.detected_frames += 1
            tv = tvecs[0][0]
            rx, ry, rz = float(tv[0]), float(tv[1]), float(tv[2])
            self.last_z = rz

            aruco.drawDetectedMarkers(
                frame, [corners[valid_idx]], np.array([[EXPECTED_ID or 0]]))
            try:
                cv2.drawFrameAxes(frame, self.K, self.D, rvecs[0], tvecs[0], 0.4)
            except Exception:
                pass

            # Kalman
            meas = np.array([rx, ry])
            if self.tracker is None:
                self.tracker = MarkerKalmanTracker(rx, ry)
                self.get_logger().info(f"Tracker init ({rx:.2f},{ry:.2f})")
            else:
                self.tracker.predict(dt)
                self.tracker.update(meas)

            self.coast_cnt = 0
            self._publish(self.tracker.position, rz,
                          self.tracker.velocity, coasting=False)

            # Speed overlay
            vx, vy = self.tracker.velocity
            speed_ms   = math.sqrt(vx**2 + vy**2)
            speed_kmh  = speed_ms * 3.6
            # EMA smooth to prevent flickering
            self.speed_kmh_smooth = (SPEED_EMA_ALPHA * speed_kmh +
                                     (1 - SPEED_EMA_ALPHA) * self.speed_kmh_smooth)

            # Landing confirmation
            horiz_err  = math.sqrt(rx**2 + ry**2)
            on_marker  = (rz < LAND_CONFIRM_ALT_M and
                          horiz_err < LAND_CONFIRM_ERR_M)

            det_pct = 100.0 * self.detected_frames / self.total_frames
            self._draw_overlay(frame, rx, ry, rz, det_pct, on_marker)

        else:
            # Coast 
            if self.tracker is not None and self.coast_cnt < MAX_COAST_FRAMES:
                pred = self.tracker.predict(dt)
                self._publish(pred, self.last_z,
                              self.tracker.velocity, coasting=True)
                self.coast_cnt += 1
                det_pct = 100.0 * self.detected_frames / self.total_frames
                cv2.putText(frame,
                    f"COASTING {self.coast_cnt}/{MAX_COAST_FRAMES}  "
                    f"spd={self.speed_kmh_smooth:.1f}km/h  det={det_pct:.0f}%",
                    (6, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,165,255), 2)
            else:
                if self.coast_cnt >= MAX_COAST_FRAMES:
                    self.tracker = None
                    self.get_logger().warn(
                        f"Tracker reset — det="
                        f"{100*self.detected_frames/max(self.total_frames,1):.1f}%")
                self.speed_kmh_smooth *= 0.9   # decay speed when lost
                cv2.putText(frame, "NO MARKER",
                    (6, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,255), 2)

        cv2.imshow("ArUco", frame)
        cv2.waitKey(1)

    def _draw_overlay(self, frame, rx, ry, rz, det_pct, on_marker):
        h = frame.shape[0]

        # Line 1: detection info
        det_color = (0, 255, 0) if not on_marker else (0, 255, 128)
        cv2.putText(frame,
            f"ID={EXPECTED_ID}  dx={rx:+.2f}m dy={ry:+.2f}m z={rz:.2f}m  "
            f"det={det_pct:.0f}%",
            (6, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, det_color, 2)

        # Line 2: vehicle speed
        spd_color = (0, 220, 255)  # yellow-cyan
        cv2.putText(frame,
            f"Vehicle speed: {self.speed_kmh_smooth:.1f} km/h",
            (6, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.65, spd_color, 2)

        # Line 3: landing confirmation (bottom of frame, large text)
        if on_marker:
            # Green banner at bottom — confirmed on marker
            cv2.rectangle(frame, (0, h-50), (frame.shape[1], h), (0,180,0), -1)
            cv2.putText(frame, "LANDED ON MARKER",
                (10, h-15), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
        elif rz < LAND_CONFIRM_ALT_M * 1.5:
            # Orange warning — close but not confirmed
            cv2.putText(frame,
                f"CLOSE: z={rz:.2f}m err={math.sqrt(rx**2+ry**2):.2f}m",
                (6, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0,165,255), 2)
          
    def _publish(self, xy, z, vel, coasting):

        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.pose.position.x = float(xy[0])
        msg.pose.position.y = float(xy[1])
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = float(vel[0])
        msg.pose.orientation.y = float(vel[1])
        msg.pose.orientation.z = 1.0 if coasting else 0.0
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)
      
def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

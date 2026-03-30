import math
import collections

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

NAN = float('nan')
# AXIS MAPPING
DIAGNOSE   = False
NORTH_AXIS = 'y'
NORTH_SIGN = -1
EAST_AXIS  = 'x'
EAST_SIGN  = +1

# FLIGHT PARAMETERS
LOOP_HZ      = 20
DT           = 1.0 / LOOP_HZ
TARGET_ALT   = 8.0
TAKEOFF_RATE = 1.0
LEAD_TIME    = 0.8  
STATIC_FORWARD_BIAS = 0.5 
#  PID gains
KP_HIGH  = 0.25
KP_LOW   = 0.40
KI_HIGH  = 0.02
KI_LOW   = 0.05
KD       = 0.15 
I_LIMIT  = 0.2  
GAIN_ALT_HIGH = 8.0
GAIN_ALT_LOW  = 2.0

#  Feedforward 
KFF = 0.15   # IF DISABLED: Kalman vel is RELATIVE (drone+car), causes oscillation

#  Smoothing 
SMOOTH_NEW = 0.25  
DEADBAND_M = 0.04

#  Speed limits 
MAX_HORIZ   = 2.5  
MAX_DESCENT = 1.0

#  Alignment gate 
# ALIGN_RADIUS_M must be clearly above the actual tracking error (~1.5m).
ALIGN_RADIUS_M   = 2.0
ALIGN_HOLD_SECS  = 1.0
ALIGN_HOLD_STEPS = int(ALIGN_HOLD_SECS * LOOP_HZ)   # = 20 frames
ALIGN_MIN_FRAC   = 0.70   

# Descent rates 
# Thresholds chosen to match actual tracking error
DESCENT_FAST  = 0.40   
DESCENT_MED   = 0.25  
DESCENT_SLOW  = 0.10   
DESCENT_PAUSE = 0.00 
DESCENT_BLIND = 2.0
#  Altitude management 
ABORT_ALT_M    = 4.0   
BLIND_ALT_M    = 2.5   
LAND_ALT_M     = 0.45   
LOST_ABORT_S   = 2.0   
ALT_SYNC_EVERY = 10    

#  Landing confirmation
# Triple condition — uniquely identifies "on car roof, on marker":
LAND_CONFIRM_ALT   = 0.8    
LAND_CONFIRM_ERR   = 0.4  
LAND_STABLE_FRAMES = 10     


class OffboardLanding(Node):

    def __init__(self):
        super().__init__('offboard_landing')

        self.declare_parameter('north_axis', NORTH_AXIS)
        self.declare_parameter('north_sign', float(NORTH_SIGN))
        self.declare_parameter('east_axis',  EAST_AXIS)
        self.declare_parameter('east_sign',  float(EAST_SIGN))
        self.declare_parameter('kff',        KFF)

        self.pose_sub     = self.create_subscription(
            PoseStamped, '/aruco_pose', self.pose_callback, 10)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub     = self.create_publisher(
            TrajectorySetpoint,  '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub      = self.create_publisher(
            VehicleCommand,      '/fmu/in/vehicle_command', 10)

        self.timer = self.create_timer(DT, self.timer_callback)

        # Marker state
        self.marker_x  = 0.0
        self.marker_y  = 0.0
        self.marker_z  = TARGET_ALT   # depth = altitude above car
        self.marker_vx = 0.0          # Kalman velocity x (feedforward)
        self.marker_vy = 0.0          # Kalman velocity y (feedforward)
        self.coasting  = False
        self.last_pose_t = self.get_clock().now()

        # Flight state
        self.phase       = "INIT"
        self.counter     = 0
        self.current_alt = 0.0

        # PID state
        self.vn_smooth  = 0.0
        self.ve_smooth  = 0.0
        self.vn_prev    = 0.0
        self.ve_prev    = 0.0
        self.integral_n = 0.0
        self.integral_e = 0.0

        # Alignment window 
        self.align_buf: collections.deque = collections.deque(
            maxlen=ALIGN_HOLD_STEPS)

        # Descent state
        self.lost_since:    float | None = None
        self.alt_sync_ctr:  int          = 0

        # Landing confirmation state
        self.land_stable_ctr = 0    # counts consecutive frames meeting conditions

        # Blind landing
        self.blind_car_vn = 0.0
        self.blind_car_ve = 0.0
        self.blind_pos_n  = 0.0
        self.blind_pos_e  = 0.0

        self.get_logger().info(
            f"OffboardLanding ready | "
            f"ALIGN={ALIGN_RADIUS_M}m/{ALIGN_HOLD_SECS}s | "
            f"BLIND={BLIND_ALT_M}m | LAND={LAND_ALT_M}m | KFF={KFF}"
        )

    def _get_axis(self):
        return (
            self.get_parameter('north_axis').value,
            float(self.get_parameter('north_sign').value),
            self.get_parameter('east_axis').value,
            float(self.get_parameter('east_sign').value),
        )

    def _cam_to_ned(self, cx: float, cy: float) -> tuple[float, float]:
        n_ax, n_sg, e_ax, e_sg = self._get_axis()
        s = {'x': cx, 'y': cy}
        return n_sg * s[n_ax], e_sg * s[e_ax]

    def _altitude_gains(self) -> tuple[float, float]:
        alt = max(GAIN_ALT_LOW, min(GAIN_ALT_HIGH, self.current_alt))
        t   = (alt - GAIN_ALT_LOW) / (GAIN_ALT_HIGH - GAIN_ALT_LOW)
        return KP_LOW + t*(KP_HIGH-KP_LOW), KI_LOW + t*(KI_HIGH-KI_LOW)

    def pose_callback(self, msg: PoseStamped):
        self.marker_x  = msg.pose.position.x
        self.marker_y  = msg.pose.position.y
        self.marker_z  = msg.pose.position.z
        self.marker_vx = msg.pose.orientation.x
        self.marker_vy = msg.pose.orientation.y
        self.coasting  = msg.pose.orientation.z > 0.5
        self.last_pose_t = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        age = (now - self.last_pose_t).nanoseconds / 1e9

        pose_fresh    = age < 0.5
        truly_visible = pose_fresh and not self.coasting
        coast_ok      = pose_fresh and self.coasting
        use_marker    = truly_visible or coast_ok

        # Continuous absence timer
        if truly_visible:
            self.lost_since = None
        elif self.lost_since is None:
            self.lost_since = now.nanoseconds / 1e9
        absent_s = (
            (now.nanoseconds / 1e9 - self.lost_since)
            if self.lost_since is not None else 0.0
        )

        # Landing confirmation check (runs every cycle regardless of phase)
        self._check_landing_confirmation(truly_visible)

        # PX4 heartbeat
        hb = OffboardControlMode()
        hb.timestamp = int(now.nanoseconds / 1000)
        hb.velocity  = True
        self.offboard_pub.publish(hb)

        # DIAGNOSE mode
        if DIAGNOSE:
            self._pub_vel_only(0.0, 0.0, 0.0)
            vn_p, ve_p = self._cam_to_ned(self.marker_x, self.marker_y)
            vn_f, ve_f = self._cam_to_ned(self.marker_vx, self.marker_vy)
            kp, ki = self._altitude_gains()
            self.get_logger().info(
                f"[DIAG] pos=({self.marker_x:+.3f},{self.marker_y:+.3f}) "
                f"vel=({self.marker_vx:+.3f},{self.marker_vy:+.3f}) "
                f"z={self.marker_z:.2f}m vis={truly_visible} | "
                f"P→({kp*vn_p:+.3f},{kp*ve_p:+.3f}) "
                f"FF→({KFF*vn_f:+.3f},{KFF*ve_f:+.3f})"
            )
            self.counter += 1
            return

        vn, ve = self._compute_cmd(use_marker)

        if   self.phase == "INIT":       self._init()
        elif self.phase == "TAKEOFF":    self._takeoff()
        elif self.phase == "ALIGN":      self._align(vn, ve, truly_visible)
        elif self.phase == "DESCEND":    self._descend(
                                             vn, ve, truly_visible,
                                             use_marker, absent_s)
        elif self.phase == "BLIND_LAND": self._blind_land()

        err = math.sqrt(self.marker_x**2 + self.marker_y**2)
        kp, _ = self._altitude_gains()
        self.get_logger().info(
            f"[{self.phase}] vis={truly_visible} err={err:.2f}m "
            f"alt={self.current_alt:.2f}m vN={vn:+.2f} vE={ve:+.2f} "
            f"Kp={kp:.2f} stable={self.land_stable_ctr}"
        )
        self.counter += 1

    # ─────────────────────────────────────────────────────────────────────────
    def _check_landing_confirmation(self, truly_visible: bool):
        err = math.sqrt(self.marker_x**2 + self.marker_y**2)

        if (truly_visible and
                self.marker_z < LAND_CONFIRM_ALT and
                err < LAND_CONFIRM_ERR):
            self.land_stable_ctr += 1
        else:
            self.land_stable_ctr = 0

        if self.land_stable_ctr >= LAND_STABLE_FRAMES and self.phase != "DONE":
            self.get_logger().info(
                f"LANDING CONFIRMED — on marker | "
                f"z={self.marker_z:.3f}m err={err:.3f}m "
                f"stable={self.land_stable_ctr}fr"
            )
            self.send_cmd(400, 0.0)   # disarm
            self.phase = "DONE"

    def _compute_cmd(self, use_marker: bool) -> tuple[float, float]:

        kp, ki = self._altitude_gains()
        kff = float(self.get_parameter('kff').value)

        mx, my = self.marker_x, self.marker_y
        vx, vy = self.marker_vx, self.marker_vy

        # 1. Calculate the normalized direction of travel for the car
        v_total = math.sqrt(vx**2 + vy**2)
        if v_total > 0.1:
            # Unit vector of car movement
            dir_x = vx / v_total
            dir_y = vy / v_total
        else:
            dir_x, dir_y = 0.0, 0.0

        # 2. Add BOTH Dynamic Lead (Velocity based) AND Static Bias (Fixed distance)
        # This pushes the target further forward along the car's path
        mx_lead = mx + (vx * LEAD_TIME) + (dir_x * STATIC_FORWARD_BIAS)
        my_lead = my + (vy * LEAD_TIME) + (dir_y * STATIC_FORWARD_BIAS)

        en, ee = self._cam_to_ned(mx_lead, my_lead)
        fn, fe = self._cam_to_ned(vx, vy)
        # LEAD COMPENSATION: Adjust this value (seconds) to look further ahead.
        # 0.3 to 0.5 is usually the sweet spot for Gazebo SITL.
        LEAD_TIME = 0.4 

        mx, my = self.marker_x, self.marker_y
        vx, vy = self.marker_vx, self.marker_vy  # Filtered marker velocity

        # 3. TARGET LEAD CALCULATION
        # We shift the target coordinates by the vehicle's velocity * LEAD_TIME.
        # This forces the PID to 'chase' a point in front of the car.
        mx_lead = mx + (vx * LEAD_TIME)
        my_lead = my + (vy * LEAD_TIME)

        # 4. ERROR CALCULATION (NED Frame)
        # Convert the 'led' marker position and marker velocity to NED
        en, ee = self._cam_to_ned(mx_lead, my_lead)
        fn, fe = self._cam_to_ned(vx, vy)

        # Apply Deadband to the error (prevents jitter when very close)
        err_dist = math.sqrt(en**2 + ee**2)
        if err_dist < DEADBAND_M:
            en, ee = 0.0, 0.0
            self.integral_n *= 0.98
            self.integral_e *= 0.98

        # 5. INTEGRAL TERM
        self.integral_n = max(-I_LIMIT, min(I_LIMIT, self.integral_n + en * DT))
        self.integral_e = max(-I_LIMIT, min(I_LIMIT, self.integral_e + ee * DT))

        # 6. VELOCITY COMMAND GENERATION (FF + P + I)
        # kff * fn: Matches the car's current speed
        # kp * en: Pulls the drone toward the 'lead' point
        vn_tgt = (kff * fn) + (kp * en) + (ki * self.integral_n)
        ve_tgt = (kff * fe) + (kp * ee) + (ki * self.integral_e)

        # 7. SMOOTHING & DERIVATIVE (D-Term)
        # Use low-pass filter to prevent jerky motor responses
        self.vn_smooth = (1 - SMOOTH_NEW) * self.vn_smooth + SMOOTH_NEW * vn_tgt
        self.ve_smooth = (1 - SMOOTH_NEW) * self.ve_smooth + SMOOTH_NEW * ve_tgt

        # D-term based on the change in smoothed velocity
        vn_out = self.vn_smooth - KD * (self.vn_smooth - self.vn_prev) / DT
        ve_out = self.ve_smooth - KD * (self.ve_smooth - self.ve_prev) / DT
        
        # Update previous values for next cycle
        self.vn_prev = self.vn_smooth
        self.ve_prev = self.ve_smooth

        # 8. GLOBAL SPEED LIMITER
        # Ensures the drone doesn't exceed its physical tilt/speed limits
        final_spd = math.sqrt(vn_out**2 + ve_out**2)
        if final_spd > MAX_HORIZ:
            vn_out *= MAX_HORIZ / final_spd
            ve_out *= MAX_HORIZ / final_spd

        return vn_out, ve_out
   
    def _init(self):
        if self.counter > 50:
            self.send_cmd(176, 1.0, 6.0)
            self.send_cmd(400, 1.0)
            self.phase = "TAKEOFF"
            self.get_logger().info("→ TAKEOFF")

    def _takeoff(self):
        self.current_alt = min(self.current_alt + TAKEOFF_RATE * DT, TARGET_ALT)
        self._pub_pos_hold(0.0, 0.0, -self.current_alt)
        if self.current_alt >= TARGET_ALT:
            self.phase = "ALIGN"
            self.get_logger().info("→ ALIGN")

    def _align(self, vn, ve, truly_visible):
        if truly_visible:
            self.current_alt = self.marker_z   # sync altitude in ALIGN

        self._pub_vel_only(vn, ve, 0.0)

        err = math.sqrt(self.marker_x**2 + self.marker_y**2)

        # Always append something — 1 if inside, 0 if outside or not visible
        inside = (truly_visible and err < ALIGN_RADIUS_M)
        self.align_buf.append(1 if inside else 0)

        n = len(self.align_buf)
        if (n >= ALIGN_HOLD_STEPS and
                sum(self.align_buf) >= int(ALIGN_MIN_FRAC * ALIGN_HOLD_STEPS)):
            self.align_buf.clear()
            self.phase = "DESCEND"
            self.get_logger().info(
                f"→ DESCEND | err={err:.2f}m alt={self.current_alt:.1f}m")

    def _descend(self, vn, ve, truly_visible, use_marker, absent_s):
        """
        Descend while tracking.
        """
        # Periodic altitude sync from sensor
        self.alt_sync_ctr += 1
        if truly_visible and self.alt_sync_ctr >= ALT_SYNC_EVERY:
            self.current_alt = self.marker_z
            self.alt_sync_ctr = 0

        err = math.sqrt(self.marker_x**2 + self.marker_y**2)

        # Abort to ALIGN: marker absent too long at high altitude
        if absent_s > LOST_ABORT_S and self.current_alt > ABORT_ALT_M:
            self.phase = "ALIGN"
            self.align_buf.clear()
            self.get_logger().warn(
                f"Absent {absent_s:.1f}s at {self.current_alt:.1f}m → ALIGN")
            return

        # Commit to blind land: lost below blind threshold
        if self.current_alt < BLIND_ALT_M and not use_marker:
            self._enter_blind()
            return

        # Adaptive descent rate based on alignment error
        if   err < 0.5:  vd = DESCENT_FAST
        elif err < 1.8:  vd = DESCENT_MED    # covers actual 1.2-1.5m error
        elif err < 2.5:  vd = DESCENT_SLOW
        else:            vd = DESCENT_PAUSE

        self.current_alt = max(self.current_alt - vd * DT, 0.0)
        self._pub_vel_only(vn, ve, vd)

        # Software land detect (backup to confirmation system)
        if self.current_alt < LAND_ALT_M:
            self.send_cmd(400, 0.0)
            self.get_logger().info(f"TOUCHDOWN (alt) at {self.current_alt:.2f}m")
            self.phase = "DONE"
            
    def _enter_blind(self):
        """Freeze the drone's absolute tracking velocity to coast matching the car."""
        # Use the drone's last smoothed PID output (absolute velocity) 
        # instead of the marker's relative velocity.
        self.blind_vn = self.vn_smooth*1.1
        self.blind_ve = self.ve_smooth*1.1
        
        self.phase = "BLIND_LAND"
        self.get_logger().warn(
            f"→ BLIND_LAND at {self.current_alt:.1f}m | "
            f"Coasting at vN={self.blind_vn:+.2f} vE={self.blind_ve:+.2f}")
    def _blind_land(self):
        """Open-loop descent using the frozen tracking velocity."""
        # Use the faster drop rate to minimize drift time
        self.current_alt = max(self.current_alt - DESCENT_BLIND * DT, 0.0)

        # Publish the frozen velocity and the faster descent rate
        self._pub_vel_only(self.blind_vn, self.blind_ve, DESCENT_BLIND)

        self.get_logger().info(
            f"BLIND_LAND alt={self.current_alt:.2f}m "
            f"vN={self.blind_vn:+.2f} vE={self.blind_ve:+.2f}")

        if self.current_alt < LAND_ALT_M:
            self.send_cmd(400, 0.0)
            self.get_logger().info("Blind touchdown — disarmed")
            self.phase = "DONE"

    def _pub_vel_only(self, vn, ve, vd):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position  = [NAN, NAN, NAN]
        msg.velocity  = [float(vn), float(ve), float(vd)]
        msg.yaw       = 0.0
        self.traj_pub.publish(msg)

    def _pub_pos_hold(self, n, e, d):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position  = [float(n), float(e), float(d)]
        msg.velocity  = [NAN, NAN, NAN]
        msg.yaw       = 0.0
        self.traj_pub.publish(msg)

    def _pub_pos_vel(self, n, e, d, vn, ve, vd):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position  = [float(n), float(e), float(d)]
        msg.velocity  = [float(vn), float(ve), float(vd)]
        msg.yaw       = 0.0
        self.traj_pub.publish(msg)

    def send_cmd(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        msg.command          = int(command)
        msg.param1           = float(p1)
        msg.param2           = float(p2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        self.cmd_pub.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = OffboardLanding()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

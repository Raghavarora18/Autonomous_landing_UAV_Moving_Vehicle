#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <cmath>

namespace gazebo
{

class CircularMotionPlugin : public ModelPlugin
{
private:
  physics::ModelPtr model;
  event::ConnectionPtr updateConnection;

  // Adjust these here, but remember to RECOMPILE (make)
  double radius = 3.0;           // Reduced radius as requested
  double angular_velocity = 0.2; // Speed of rotation

public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
  {
    this->model = _model;
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&CircularMotionPlugin::OnUpdate, this));
  }

  void OnUpdate()
  {
    // 1. Calculate the linear speed needed for this radius
    double linear_speed = radius * angular_velocity;

    // 2. GET CURRENT ORIENTATION (The "Fix" for Orientation)
    // We get the car's current rotation in the world so we know which way is "Forward"
    ignition::math::Pose3d currentPose = this->model->WorldPose();
    ignition::math::Quaterniond currentRot = currentPose.Rot();

    // 3. CREATE LOCAL FORWARD VELOCITY
    // This is "Forward" relative to the car's nose (X-axis)
    ignition::math::Vector3d localLinearVel(linear_speed, 0, 0);

    // 4. TRANSFORM TO WORLD FRAME
    // This rotates the "Forward" vector to match the car's current heading
    ignition::math::Vector3d worldLinearVel = currentRot.RotateVector(localLinearVel);

    // 5. PRESERVE GRAVITY (The "Fix" for Grounding)
    // Instead of setting Z to 0 (which makes it float), we keep the current 
    // vertical velocity calculated by the physics engine.
    worldLinearVel.Z() = this->model->WorldLinearVel().Z();

    // 6. APPLY TO MODEL
    this->model->SetLinearVel(worldLinearVel);
    this->model->SetAngularVel(ignition::math::Vector3d(0, 0, angular_velocity));
  }
};
GZ_REGISTER_MODEL_P

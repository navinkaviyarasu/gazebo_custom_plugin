#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Sensor.hh>

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Pose.hh> // Needed for link poses
#include <gz/sim/components/JointPosition.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h> // Using Double_V for a 2-vector of servo lengths

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <cmath>
#include <algorithm>
#include <gz/plugin/Register.hh>

using namespace gz;
using namespace sim;
using namespace systems;

// --- Kinematic Parameters in Meters ---
// Servo fixed anchor points (on base_link/gimbal_mount)
const math::Vector3d ANCHOR_1( 0.0,  0.06731, 0.2525); // Y-axis offset for Roll control
const math::Vector3d ANCHOR_2( 0.06731, 0.0, 0.2525);  // X-axis offset for Pitch control

// Servo clevis points (on motor_housing link) in the neutral (0 deg) frame.
// These are relative to the model origin (0,0,0) as per the SDF link poses.
const math::Vector3d CLEVIS_1_NEUTRAL( 0.0, 0.0279, 0.1675); // Y-offset clevis
const math::Vector3d CLEVIS_2_NEUTRAL( 0.0279, 0.0, 0.1675); // X-offset clevis

// Pivot point center (used for relative calculations)
const math::Vector3d PIVOT_POINT( 0.0, 0.0, 0.1675);

// --- MightyZap 12LF-12PT-27 Servo Physical Limits (approx) ---
const double SERVO_MIN_LEN = 0.085; // 85 mm
const double SERVO_MAX_LEN = 0.115; // 115 mm

class TVCPlugin : public System,
                  public ISystemConfigure,
                  public ISystemPreUpdate
{
public:
    TVCPlugin() :
        servo1_current_len_(0.0),
        servo2_current_len_(0.0),
        servo1_target_len_(0.0),
        servo2_target_len_(0.0)
    {
    }

    // ISystemConfigure
    virtual void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override
    {
        this->model_ = Model(_entity);
        if (!this->model_.Valid(_ecm))
        {
            gzerr << "TVCPlugin must be attached to a model entity." << std::endl;
            return;
        }

        // --- Get parameters from SDF ---
        // MightyZap 12LF-12PT-27: 112 mm/s = 0.112 m/s
        this->servo_max_speed_ = _sdf->Get<double>("servo_max_speed", 0.112).first; // m/s
        
        // --- Joint/Link setup ---
        std::string roll_joint_name = _sdf->Get<std::string>("roll_joint", "servo0_roll_joint").first;
        std::string pitch_joint_name = _sdf->Get<std::string>("pitch_joint", "servo1_pitch_joint").first; // Universal joint uses 'pitch' on top of 'roll'
        std::string engine_link_name = _sdf->Get<std::string>("engine_link", "motor_housing").first; // Using motor_housing for thrust application

        this->roll_joint_ = this->model_.JointByName(_ecm, roll_joint_name);
        this->pitch_joint_ = this->model_.JointByName(_ecm, pitch_joint_name);
        this->engine_link_ = this->model_.LinkByName(_ecm, engine_link_name);
        
        // Get base_link for anchor transform (base_link/gimbal_mount is fixed to the rocket body)
        this->base_link_ = this->model_.LinkByName(_ecm, "base_link"); 

        if (this->roll_joint_ == kNullEntity || this->pitch_joint_ == kNullEntity || this->engine_link_ == kNullEntity || this->base_link_ == kNullEntity)
        {
            gzerr << "Could not find all required joints/links (roll_joint, pitch_joint, motor_housing, base_link). Check SDF names." << std::endl;
            return;
        }
        
        // Initialize neutral length. The distance between Anchor and Clevis at 0-angle.
        // Anchor 1 (0, 0.06731, 0.2525) to Clevis 1 (0, 0.0279, 0.1675)
        math::Vector3d delta1 = ANCHOR_1 - CLEVIS_1_NEUTRAL;
        // Anchor 2 (0.06731, 0, 0.2525) to Clevis 2 (0.0279, 0, 0.1675)
        math::Vector3d delta2 = ANCHOR_2 - CLEVIS_2_NEUTRAL;

        this->servo1_current_len_ = delta1.Length(); // L0 for Servo 1 (~0.0935 m)
        this->servo2_current_len_ = delta2.Length(); // L0 for Servo 2 (~0.0935 m)
        this->servo1_target_len_ = this->servo1_current_len_;
        this->servo2_target_len_ = this->servo2_current_len_;

        // --- Setup Gazebo Transport Subscriber ---
        std::string topic_name = _sdf->Get<std::string>("topic", "/tvc_servo_command").first;
        this->gz_node_.Subscribe(topic_name, &TVCPlugin::OnServoCommand, this);
        
        gzmsg << "TVCPlugin configured for TVC model. Max Servo Speed: " << this->servo_max_speed_ << " m/s." << std::endl;
    }

    // ISystemPreUpdate
    virtual void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override
    {
        // Don't run if simulation is paused or physics isn't on
        // if (_info.paused || !_info.simTime)
        if (_info.paused || _info.simTime.count() == 0)

        {
            return;
        }

        double dt = std::chrono::duration<double>(_info.dt).count();

        // --- 1. Apply Actuator Dynamics (Slew Rate) ---
        // Rate limit the servo length command by max linear speed
        double max_change = this->servo_max_speed_ * dt;
        
        double s1_error = this->servo1_target_len_ - this->servo1_current_len_;
        this->servo1_current_len_ += std::clamp(s1_error, -max_change, max_change);
        
        double s2_error = this->servo2_target_len_ - this->servo2_current_len_;
        this->servo2_current_len_ += std::clamp(s2_error, -max_change, max_change);

        // --- 2. Inverse Kinematics (Servo Lengths -> Gimbal Angles) ---
        
        // Use the initial calculated neutral lengths (L0)
        math::Vector3d delta1_neutral = ANCHOR_1 - CLEVIS_1_NEUTRAL;
        double L0_1 = delta1_neutral.Length();
        
        math::Vector3d delta2_neutral = ANCHOR_2 - CLEVIS_2_NEUTRAL;
        double L0_2 = delta2_neutral.Length();

        // Effective moment arm for Roll/Pitch based on geometry.
        // We use an average of the anchor/clevis lateral offsets for a simple linear model.
        double r_roll = (ANCHOR_1.Y() + CLEVIS_1_NEUTRAL.Y()) / 2.0; 
        double r_pitch = (ANCHOR_2.X() + CLEVIS_2_NEUTRAL.X()) / 2.0; 
        
        // Approximate Kinematics (Change in length -> Angle)
        // delta_L ~ r * theta => theta ~ delta_L / r
        // Note the negative sign for roll to align length extension with positive joint angle
        double roll_angle = -(this->servo1_current_len_ - L0_1) / r_roll; // Servo 1 controls Roll (X-axis rotation)
        double pitch_angle = (this->servo2_current_len_ - L0_2) / r_pitch; // Servo 2 controls Pitch (Y-axis rotation)

        // Clamp angles to joint limits (-0.25 to 0.25 rad, from SDF)
        roll_angle = std::clamp(roll_angle, -0.25, 0.25);
        pitch_angle = std::clamp(pitch_angle, -0.25, 0.25);

        // --- 3. Set Joint Positions ---
        _ecm.SetComponentData<components::JointPosition>(this->roll_joint_, {roll_angle});
        _ecm.SetComponentData<components::JointPosition>(this->pitch_joint_, {pitch_angle});
        // _ecm.SetComponentData<components::JointPositionCmd>(this->roll_joint_, roll_angle);
        // _ecm.SetComponentData<components::JointPositionCmd>(this->pitch_joint_, pitch_angle);

        
        // --- 4. Thrust ---
        // Thrust is handled externally by gz-sim-multicopter-motor-model-system, no action needed here.
    }

    // Callback for Gazebo Transport topic
    void OnServoCommand(const msgs::Double_V &_msg)
    {
        if (_msg.data_size() < 2)
        {
            gzerr << "Received servo command with < 2 data elements." << std::endl;
            return;
        }
        
        // Assuming your LQR controller outputs the desired *length* for each servo.
        // Value 0: Desired length for Servo 1 (controls Roll)
        // Value 1: Desired length for Servo 2 (controls Pitch)
        
        // Clamp the commanded length to the physical limits of the MightyZap 12LF-12PT-27
        this->servo1_target_len_ = std::clamp(_msg.data(0), SERVO_MIN_LEN, SERVO_MAX_LEN);
        this->servo2_target_len_ = std::clamp(_msg.data(1), SERVO_MIN_LEN, SERVO_MAX_LEN);
    }

private:
    Model model_{kNullEntity};
    Entity roll_joint_{kNullEntity};
    Entity pitch_joint_{kNullEntity};
    Entity engine_link_{kNullEntity};
    Entity base_link_{kNullEntity};
    
    transport::Node gz_node_;
    
    // --- Parameters ---
    double servo_max_speed_; // m/s (0.112 m/s for MightyZap 12LF-12PT-27)
    
    // --- State ---
    double servo1_current_len_; // The "real" simulated length
    double servo2_current_len_;
    double servo1_target_len_;  // The "commanded" length from Python
    double servo2_target_len_;
};

// Register the plugin
// GZ_PLUGIN_REGISTER_CLASS(
//     TVCPlugin
// )

GZ_ADD_PLUGIN(
  TVCPlugin,
  gz::sim::System,
  TVCPlugin::ISystemConfigure,
  TVCPlugin::ISystemPreUpdate
)

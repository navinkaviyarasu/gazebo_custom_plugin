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
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Pose.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Matrix3.hh>
#include <cmath>
#include <algorithm>
#include <memory>

using namespace gz;
using namespace sim;
using namespace systems;

// --- Input Format Enumeration ---
enum class InputFormat {
    NORMALIZED_MINUS1_1,  // PX4 normalized [-1, 1] range (DEFAULT)
    PWM_1000_2000,        // Standard RC PWM range
    SERVO_LENGTH,         // Direct servo length command
    JOINT_ANGLE           // Direct joint angle
};

// --- Real Gimbal Geometry Parameters ---
// All dimensions in meters, relative to gimbal center of rotation
struct GimbalGeometry {
    double xB;        // X-distance to tail stroke clevis point B
    double zB;        // Z-distance to tail stroke clevis point B
    double yE;        // Y-distance to rod-end clevis point E
    double zE;        // Z-distance to rod-end clevis point E
    double L4;        // Distance from origin to C (along Z)
    double L5;        // Distance from C to D (along body X) - servo 1 arm
    double L7;        // Distance from C to F (along body Y) - servo 2 arm
};

// --- Servo Calibration Parameters ---
// For converting PWM/normalized input to servo stroke lengths
struct ServoCalibration {
    double a3;        // Stroke slope for servo 1 (L3 per unit PWM)
    double a6;        // Stroke slope for servo 2 (L6 per unit PWM)
    double p01;       // PWM neutral value for servo 1
    double p02;       // PWM neutral value for servo 2
    double L3_min;    // Minimum stroke length for servo 1
    double L3_max;    // Maximum stroke length for servo 1
    double L6_min;    // Minimum stroke length for servo 2
    double L6_max;    // Maximum stroke length for servo 2
};

// --- Default Kinematic Parameters ---
// MightyZap 12LF-12PT-27 Servo Physical Limits
const double SERVO_MIN_LEN = 0.085; // 85 mm
const double SERVO_MAX_LEN = 0.115; // 115 mm

// Default Viper gimbal geometry (from pwm_to_angles.py)
const GimbalGeometry DEFAULT_GEOMETRY = {
    .xB = 0.06741,   // 67.41 mm
    .zB = 0.0917,    // 91.7 mm
    .yE = 0.06741,   // 67.41 mm
    .zE = 0.0917,    // 91.7 mm
    .L4 = 0.0228,    // 22.8 mm
    .L5 = 0.028,     // 28.0 mm
    .L7 = 0.028      // 28.0 mm
};

// Default servo calibration (27mm stroke MightyZap)
const ServoCalibration DEFAULT_CALIBRATION = {
    .a3 = 0.0225,    // 22.5 mm per 1000 µs PWM
    .a6 = 0.0225,    // 22.5 mm per 1000 µs PWM
    .p01 = 1500.0,   // Neutral PWM
    .p02 = 1500.0,   // Neutral PWM
    .L3_min = SERVO_MIN_LEN,
    .L3_max = SERVO_MAX_LEN,
    .L6_min = SERVO_MIN_LEN,
    .L6_max = SERVO_MAX_LEN
};

// --- Kinematics Helper Functions ---

// Rotation matrix about X axis (roll): Rx(theta)
math::Matrix3d RotationX(double theta)
{
    double c = std::cos(theta), s = std::sin(theta);
    return math::Matrix3d(1, 0, 0, 0, c, -s, 0, s, c);
}

// Rotation matrix about Y axis (pitch): Ry(phi)
math::Matrix3d RotationY(double phi)
{
    double c = std::cos(phi), s = std::sin(phi);
    return math::Matrix3d(c, 0, s, 0, 1, 0, -s, 0, c);
}

// Compute body axes X, Y, Z given gimbal angles
// Returns the rotation matrix for the gimbal body frame
math::Matrix3d GetBodyAxesMatrix(double theta, double phi)
{
    return RotationY(phi) * RotationX(theta);
}

// Compute all geometric points given gimbal angles and geometry
struct GeometricPoints {
    math::Vector3d A, B, C, D, E, F;
    math::Vector3d X, Y, Z;  // Body axes
};

GeometricPoints ComputePoints(double theta, double phi, const GimbalGeometry &geom)
{
    GeometricPoints pts;
    math::Matrix3d R = GetBodyAxesMatrix(theta, phi);
    
    // Extract body axes from rotation matrix columns
    pts.X = math::Vector3d(R(0,0), R(1,0), R(2,0));
    pts.Y = math::Vector3d(R(0,1), R(1,1), R(2,1));
    pts.Z = math::Vector3d(R(0,2), R(1,2), R(2,2));
    
    pts.A = math::Vector3d(0.0, 0.0, 0.0);              // Origin
    pts.B = math::Vector3d(geom.xB, 0.0, geom.zB);     // Tail clevis
    pts.C = pts.A - geom.L4 * pts.Z;                    // C point along Z
    pts.D = pts.C + geom.L5 * pts.X;                    // Servo 1 connection point
    pts.E = math::Vector3d(0.0, geom.yE, geom.zE);     // Rod-end clevis
    pts.F = pts.C + geom.L7 * pts.Y;                    // Servo 2 connection point
    
    return pts;
}

// Compute servo lengths given gimbal angles
void ComputeServoLengths(double theta, double phi, const GimbalGeometry &geom,
                         double &L3, double &L6)
{
    GeometricPoints pts = ComputePoints(theta, phi, geom);
    L3 = (pts.D - pts.B).Length();  // Servo 1 length (pitch)
    L6 = (pts.F - pts.E).Length();  // Servo 2 length (roll)
}

// Compute neutral servo lengths (at theta=0, phi=0)
void ComputeNeutralLengths(const GimbalGeometry &geom,
                          double &L3_neutral, double &L6_neutral)
{
    ComputeServoLengths(0.0, 0.0, geom, L3_neutral, L6_neutral);
}

// Compute Jacobian matrix at given angles
// J[0,0] = dL3/dtheta, J[0,1] = dL3/dphi
// J[1,0] = dL6/dtheta, J[1,1] = dL6/dphi
math::Matrix3d ComputeLengthJacobian(double theta, double phi, const GimbalGeometry &geom)
{
    double epsilon = 1e-6;
    double L3_0, L6_0, L3_th, L6_th, L3_ph, L6_ph;
    
    // Nominal
    ComputeServoLengths(theta, phi, geom, L3_0, L6_0);
    
    // Perturb theta
    ComputeServoLengths(theta + epsilon, phi, geom, L3_th, L6_th);
    
    // Perturb phi
    ComputeServoLengths(theta, phi + epsilon, geom, L3_ph, L6_ph);
    
    double dL3_dth = (L3_th - L3_0) / epsilon;
    double dL6_dth = (L6_th - L6_0) / epsilon;
    double dL3_dph = (L3_ph - L3_0) / epsilon;
    double dL6_dph = (L6_ph - L6_0) / epsilon;
    
    // Return 2x2 Jacobian as 3x3 (ignore unused elements)
    return math::Matrix3d(dL3_dth, dL3_dph, 0,
                          dL6_dth, dL6_dph, 0,
                          0, 0, 1);
}

// Inverse kinematics using Levenberg-Marquardt
// Given desired servo lengths, find gimbal angles (theta, phi)
bool InverseKinematics(double L3_target, double L6_target,
                      const GimbalGeometry &geom,
                      double &theta_out, double &phi_out,
                      double theta_init = 0.0, double phi_init = 0.0,
                      int max_iters = 30, double tol = 1e-9)
{
    double theta = theta_init, phi = phi_init;
    double lambda = 1e-4;  // Levenberg-Marquardt damping coefficient
    
    for (int iter = 0; iter < max_iters; ++iter)
    {
        double L3, L6;
        ComputeServoLengths(theta, phi, geom, L3, L6);
        
        double r3 = L3 - L3_target;
        double r6 = L6 - L6_target;
        double residual_norm = std::sqrt(r3*r3 + r6*r6);
        
        if (residual_norm < tol)
        {
            theta_out = theta;
            phi_out = phi;
            return true;
        }
        
        math::Matrix3d J = ComputeLengthJacobian(theta, phi, geom);
        
        // J^T J + lambda*I
        double JtJ_00 = J(0,0)*J(0,0) + J(1,0)*J(1,0) + lambda;
        double JtJ_01 = J(0,0)*J(0,1) + J(1,0)*J(1,1);
        double JtJ_10 = JtJ_01;
        double JtJ_11 = J(0,1)*J(0,1) + J(1,1)*J(1,1) + lambda;
        
        // J^T r
        double Jtr_0 = J(0,0)*r3 + J(1,0)*r6;
        double Jtr_1 = J(0,1)*r3 + J(1,1)*r6;
        
        // Solve [J^T J + lambda*I] * step = -J^T r
        double det = JtJ_00*JtJ_11 - JtJ_01*JtJ_10;
        if (std::abs(det) < 1e-12)
        {
            return false;  // Singular matrix
        }
        
        double step_0 = -(JtJ_11*Jtr_0 - JtJ_01*Jtr_1) / det;
        double step_1 = -(JtJ_00*Jtr_1 - JtJ_10*Jtr_0) / det;
        
        double step_norm = std::sqrt(step_0*step_0 + step_1*step_1);
        if (step_norm < tol)
        {
            theta_out = theta;
            phi_out = phi;
            return true;
        }
        
        theta += step_0;
        phi += step_1;
    }
    
    // Return best effort even if not converged
    theta_out = theta;
    phi_out = phi;
    return false;
}

class TVCPlugin : public System,
                  public ISystemConfigure,
                  public ISystemPreUpdate,
                  public ISystemPostUpdate
{
public:
    TVCPlugin() :
        servo1_current_len_(0.0),
        servo2_current_len_(0.0),
        servo1_target_len_(0.0),
        servo2_target_len_(0.0),
        servo1_velocity_(0.0),
        servo2_velocity_(0.0),
        servo_time_constant_(0.0),
        servo_natural_freq_(5.0),
        servo_damping_ratio_(0.7),
        dynamics_model_("first_order"),
        servo_min_len_(SERVO_MIN_LEN),
        servo_max_len_(SERVO_MAX_LEN),
        input_mode_("joint_position"),
        input_format_(InputFormat::NORMALIZED_MINUS1_1),
        servo_0_topic_("/gazebo/servo_0"),
        servo_1_topic_("/gazebo/servo_1"),
        transport_topic_("/tvc_servo_command"),
        servo_0_command_(0.0),
        servo_1_command_(0.0),
        gimbal_geometry_(DEFAULT_GEOMETRY),
        servo_calibration_(DEFAULT_CALIBRATION)
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
        // First-order time constant (seconds) for servo dynamics. If <= 0, plugin
        // falls back to the original rate-limiter behavior.
        this->servo_time_constant_ = _sdf->Get<double>("servo_time_constant", 0.1).first; // seconds
        // Read servo physical limits from SDF (fall back to compile-time constants)
        this->servo_min_len_ = _sdf->Get<double>("servo_min_len", SERVO_MIN_LEN).first;
        this->servo_max_len_ = _sdf->Get<double>("servo_max_len", SERVO_MAX_LEN).first;
        
        // --- Dynamics Model Selection ---
        // Choose between "first_order" (default) or "second_order"
        this->dynamics_model_ = _sdf->Get<std::string>("dynamics_model", "first_order").first;
        
        // --- Second-order system parameters ---
        // Natural frequency (rad/s) for second-order model. Typical range: 2-10 rad/s
        this->servo_natural_freq_ = _sdf->Get<double>("servo_natural_freq", 5.0).first;
        // Damping ratio (dimensionless). 0.7 is slightly underdamped (minimal overshoot)
        // <1: underdamped, =1: critically damped, >1: overdamped
        this->servo_damping_ratio_ = _sdf->Get<double>("servo_damping_ratio", 0.7).first;
        
        // Validate dynamics model selection
        if (this->dynamics_model_ != "first_order" && this->dynamics_model_ != "second_order")
        {
            gzwarn << "Invalid dynamics_model: '" << this->dynamics_model_ 
                   << "'. Using 'first_order' as default." << std::endl;
            this->dynamics_model_ = "first_order";
        }
        
        // --- Input Mode Configuration ---
        this->input_mode_ = _sdf->Get<std::string>("input_mode", "joint_position").first;
        std::string input_format_str = _sdf->Get<std::string>("input_format", "normalized_minus1_1").first;
        
        if (input_format_str == "normalized_minus1_1")
            this->input_format_ = InputFormat::NORMALIZED_MINUS1_1;
        else if (input_format_str == "pwm_1000_2000")
            this->input_format_ = InputFormat::PWM_1000_2000;
        else if (input_format_str == "servo_length")
            this->input_format_ = InputFormat::SERVO_LENGTH;
        else if (input_format_str == "joint_angle")
            this->input_format_ = InputFormat::JOINT_ANGLE;
        else
        {
            gzwarn << "Unknown input_format: '" << input_format_str << "'. Using 'normalized_minus1_1'." << std::endl;
            this->input_format_ = InputFormat::NORMALIZED_MINUS1_1;
        }
        
        // --- Servo Topic Configuration ---
        this->servo_0_topic_ = _sdf->Get<std::string>("servo_0_topic", "/gazebo/servo_0").first;
        this->servo_1_topic_ = _sdf->Get<std::string>("servo_1_topic", "/gazebo/servo_1").first;
        this->transport_topic_ = _sdf->Get<std::string>("transport_topic", "/tvc_servo_command").first;
        
        // --- Gimbal Geometry Configuration ---
        this->gimbal_geometry_.xB = _sdf->Get<double>("gimbal_xB", DEFAULT_GEOMETRY.xB).first;
        this->gimbal_geometry_.zB = _sdf->Get<double>("gimbal_zB", DEFAULT_GEOMETRY.zB).first;
        this->gimbal_geometry_.yE = _sdf->Get<double>("gimbal_yE", DEFAULT_GEOMETRY.yE).first;
        this->gimbal_geometry_.zE = _sdf->Get<double>("gimbal_zE", DEFAULT_GEOMETRY.zE).first;
        this->gimbal_geometry_.L4 = _sdf->Get<double>("gimbal_L4", DEFAULT_GEOMETRY.L4).first;
        this->gimbal_geometry_.L5 = _sdf->Get<double>("gimbal_L5", DEFAULT_GEOMETRY.L5).first;
        this->gimbal_geometry_.L7 = _sdf->Get<double>("gimbal_L7", DEFAULT_GEOMETRY.L7).first;
        
        // --- Servo Calibration Configuration ---
        this->servo_calibration_.a3 = _sdf->Get<double>("servo_a3", DEFAULT_CALIBRATION.a3).first;
        this->servo_calibration_.a6 = _sdf->Get<double>("servo_a6", DEFAULT_CALIBRATION.a6).first;
        this->servo_calibration_.p01 = _sdf->Get<double>("servo_p01", DEFAULT_CALIBRATION.p01).first;
        this->servo_calibration_.p02 = _sdf->Get<double>("servo_p02", DEFAULT_CALIBRATION.p02).first;
        this->servo_calibration_.L3_min = _sdf->Get<double>("servo_L3_min", DEFAULT_CALIBRATION.L3_min).first;
        this->servo_calibration_.L3_max = _sdf->Get<double>("servo_L3_max", DEFAULT_CALIBRATION.L3_max).first;
        this->servo_calibration_.L6_min = _sdf->Get<double>("servo_L6_min", DEFAULT_CALIBRATION.L6_min).first;
        this->servo_calibration_.L6_max = _sdf->Get<double>("servo_L6_max", DEFAULT_CALIBRATION.L6_max).first;
        
        // --- Joint/Link setup (kept for legacy compatibility, but not used) ---
        std::string roll_joint_name = _sdf->Get<std::string>("roll_joint", "servo0_roll_joint").first;
        std::string pitch_joint_name = _sdf->Get<std::string>("pitch_joint", "servo1_pitch_joint").first;
        std::string engine_link_name = _sdf->Get<std::string>("engine_link", "motor_housing").first;

        this->roll_joint_ = this->model_.JointByName(_ecm, roll_joint_name);
        this->pitch_joint_ = this->model_.JointByName(_ecm, pitch_joint_name);
        this->engine_link_ = this->model_.LinkByName(_ecm, engine_link_name);
        
        // Get base_link for anchor transform (base_link/gimbal_mount is fixed to the rocket body)
        this->base_link_ = this->model_.LinkByName(_ecm, "base_link"); 

        if (this->engine_link_ == kNullEntity || this->base_link_ == kNullEntity)
        {
            gzerr << "Could not find required links (motor_housing, base_link). Check SDF names." << std::endl;
            return;
        }
        
        // --- Angle Output Topics (for JointPositionController) ---
        this->roll_angle_topic_ = _sdf->Get<std::string>("roll_angle_topic", "/gazebo/gimbal/roll_angle").first;
        this->pitch_angle_topic_ = _sdf->Get<std::string>("pitch_angle_topic", "/gazebo/gimbal/pitch_angle").first;
        
        // Create publishers for angle setpoints
        auto roll_pub = this->gz_node_.Advertise<msgs::Double>(this->roll_angle_topic_);
        auto pitch_pub = this->gz_node_.Advertise<msgs::Double>(this->pitch_angle_topic_);
        this->roll_angle_pub_ = std::make_unique<transport::Node::Publisher>(roll_pub);
        this->pitch_angle_pub_ = std::make_unique<transport::Node::Publisher>(pitch_pub);
        
        gzmsg << "TVCPlugin angle topic setup:" << std::endl;
        gzmsg << "  Roll angle topic: " << this->roll_angle_topic_ << std::endl;
        gzmsg << "  Pitch angle topic: " << this->pitch_angle_topic_ << std::endl;
        
        // Initialize neutral lengths using real gimbal geometry
        ComputeNeutralLengths(this->gimbal_geometry_, 
                             this->servo1_current_len_, 
                             this->servo2_current_len_);
        this->servo1_target_len_ = this->servo1_current_len_;
        this->servo2_target_len_ = this->servo2_current_len_;

        // --- Setup Gazebo Transport Subscribers ---
        if (this->input_mode_ == "joint_position")
        {
            // Subscribe to PX4 servo command topics
            this->servo_0_node_ = std::make_unique<transport::Node>();
            this->servo_1_node_ = std::make_unique<transport::Node>();
            this->servo_0_node_->Subscribe(this->servo_0_topic_, &TVCPlugin::OnServo0Command, this);
            this->servo_1_node_->Subscribe(this->servo_1_topic_, &TVCPlugin::OnServo1Command, this);
            gzmsg << "TVCPlugin input mode: joint_position" << std::endl;
            gzmsg << "  Servo 0 topic: " << this->servo_0_topic_ << std::endl;
            gzmsg << "  Servo 1 topic: " << this->servo_1_topic_ << std::endl;
        }
        else if (this->input_mode_ == "transport_topic")
        {
            // Subscribe to custom transport topic (backward compatible)
            this->gz_node_.Subscribe(this->transport_topic_, &TVCPlugin::OnServoCommand, this);
            gzmsg << "TVCPlugin input mode: transport_topic (legacy)" << std::endl;
            gzmsg << "  Topic: " << this->transport_topic_ << std::endl;
        }
        else
        {
            gzwarn << "Unknown input_mode: '" << this->input_mode_ << "'. Using 'joint_position'." << std::endl;
            this->servo_0_node_ = std::make_unique<transport::Node>();
            this->servo_1_node_ = std::make_unique<transport::Node>();
            this->servo_0_node_->Subscribe(this->servo_0_topic_, &TVCPlugin::OnServo0Command, this);
            this->servo_1_node_->Subscribe(this->servo_1_topic_, &TVCPlugin::OnServo1Command, this);
        }
        
          gzmsg << "TVCPlugin configured for TVC model. Max Servo Speed: " << this->servo_max_speed_
              << " m/s. Servo time-constant: " << this->servo_time_constant_ << " s."
              << " MinLen: " << this->servo_min_len_ << " m MaxLen: " << this->servo_max_len_ << " m." << std::endl;
          
          gzmsg << "Dynamics Model: " << this->dynamics_model_ << std::endl;
          if (this->dynamics_model_ == "second_order")
          {
              gzmsg << "  Natural Frequency: " << this->servo_natural_freq_ 
                    << " rad/s, Damping Ratio: " << this->servo_damping_ratio_ << std::endl;
          }
          
          gzmsg << "Gimbal Geometry Configuration:" << std::endl;
          gzmsg << "  xB (tail clevis X): " << (this->gimbal_geometry_.xB * 1000) << " mm" << std::endl;
          gzmsg << "  zB (tail clevis Z): " << (this->gimbal_geometry_.zB * 1000) << " mm" << std::endl;
          gzmsg << "  yE (rod-end clevis Y): " << (this->gimbal_geometry_.yE * 1000) << " mm" << std::endl;
          gzmsg << "  zE (rod-end clevis Z): " << (this->gimbal_geometry_.zE * 1000) << " mm" << std::endl;
          gzmsg << "  L4 (origin to C): " << (this->gimbal_geometry_.L4 * 1000) << " mm" << std::endl;
          gzmsg << "  L5 (servo 1 arm): " << (this->gimbal_geometry_.L5 * 1000) << " mm" << std::endl;
          gzmsg << "  L7 (servo 2 arm): " << (this->gimbal_geometry_.L7 * 1000) << " mm" << std::endl;
          
          gzmsg << "Servo Calibration:" << std::endl;
          gzmsg << "  a3 (servo 1 slope): " << this->servo_calibration_.a3 << " m/PWM" << std::endl;
          gzmsg << "  a6 (servo 2 slope): " << this->servo_calibration_.a6 << " m/PWM" << std::endl;
          gzmsg << "  p01 (servo 1 neutral): " << this->servo_calibration_.p01 << " PWM" << std::endl;
          gzmsg << "  p02 (servo 2 neutral): " << this->servo_calibration_.p02 << " PWM" << std::endl;
          gzmsg << "  L3 range: [" << this->servo_calibration_.L3_min << ", " 
                << this->servo_calibration_.L3_max << "] m" << std::endl;
          gzmsg << "  L6 range: [" << this->servo_calibration_.L6_min << ", " 
                << this->servo_calibration_.L6_max << "] m" << std::endl;

          // Debug: print entity ids for the joints/links we found so it's clear we're
          // operating on the expected entities.
          gzmsg << "TVCPlugin entities - roll_joint: " << this->roll_joint_
              << ", pitch_joint: " << this->pitch_joint_
              << ", engine_link: " << this->engine_link_
              << ", base_link: " << this->base_link_ << std::endl;
    }

    // ISystemPreUpdate
    virtual void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override
    {
        // Don't run if simulation is paused or simulation time is zero
        if (_info.paused || _info.simTime == std::chrono::steady_clock::duration::zero())
        {
            return;
        }

        double dt = std::chrono::duration<double>(_info.dt).count();

        // --- 1. Apply Actuator Dynamics (First or Second-order model) ---
        if (this->dynamics_model_ == "second_order")
        {
            // Second-order system dynamics
            this->ApplySecondOrderDynamics(this->servo1_current_len_, this->servo1_velocity_, 
                                          this->servo1_target_len_, dt);
            this->ApplySecondOrderDynamics(this->servo2_current_len_, this->servo2_velocity_, 
                                          this->servo2_target_len_, dt);
        }
        else
        {
            // First-order system dynamics (default)
            this->ApplyFirstOrderDynamics(this->servo1_current_len_, this->servo1_velocity_,
                                         this->servo1_target_len_, dt);
            this->ApplyFirstOrderDynamics(this->servo2_current_len_, this->servo2_velocity_,
                                         this->servo2_target_len_, dt);
        }

        // STABILITY DEADZONE: Snap servo lengths to target if within tolerance
        // This prevents oscillation from floating-point precision errors
        // when servo length is very close to target but not exactly equal
        const double servo_length_deadzone = 1e-6;  // 1 micrometer deadzone
        if (std::abs(this->servo1_current_len_ - this->servo1_target_len_) < servo_length_deadzone)
        {
            this->servo1_current_len_ = this->servo1_target_len_;
            this->servo1_velocity_ = 0.0;  // Zero velocity when snapped to target
        }
        if (std::abs(this->servo2_current_len_ - this->servo2_target_len_) < servo_length_deadzone)
        {
            this->servo2_current_len_ = this->servo2_target_len_;
            this->servo2_velocity_ = 0.0;  // Zero velocity when snapped to target
        }

        // --- 2. Inverse Kinematics (Servo Lengths -> Gimbal Angles) ---
        // Use real gimbal kinematics with Levenberg-Marquardt solver
        double theta = 0.0;  // Roll angle (X axis)
        double phi = 0.0;    // Pitch angle (Y axis)
        
        // Call the inverse kinematics solver
        // Servo 1 (servo1_current_len_) controls pitch (L3)
        // Servo 2 (servo2_current_len_) controls roll (L6)
        bool ik_converged = InverseKinematics(
            this->servo1_current_len_,      // L3_target (pitch servo)
            this->servo2_current_len_,      // L6_target (roll servo)
            this->gimbal_geometry_,
            theta,                          // Output: roll angle
            phi,                            // Output: pitch angle
            this->last_roll_angle_,         // Initial guess from previous iteration
            this->last_pitch_angle_
        );
        
        if (!ik_converged)
        {
            // Log convergence issue but only occasionally to avoid spam
            // Uncomment for debugging:
            // gzmsg << "TVCPlugin: InverseKinematics did not converge. "
            //       << "L3=" << this->servo1_current_len_
            //       << ", L6=" << this->servo2_current_len_ << std::endl;
        }

        // Clamp angles to joint limits for safety
        // ±0.25 rad is a typical gimbal limit; adjust based on your model
        theta = std::clamp(theta, -0.25, 0.25);
        phi = std::clamp(phi, -0.25, 0.25);

        // Anti-jitter filtering: if angle change is very small, keep previous value
        // This prevents oscillation from IK numerical precision issues
        const double jitter_threshold = 1e-4;  // 0.01 degrees (increased from 1e-5 for more stability)
        if (std::abs(theta - this->last_roll_angle_) < jitter_threshold)
        {
            theta = this->last_roll_angle_;
        }
        if (std::abs(phi - this->last_pitch_angle_) < jitter_threshold)
        {
            phi = this->last_pitch_angle_;
        }

        // Rate limiter: prevent sudden angle jumps that excite oscillation
        // Max angular velocity: 0.5 rad/s (reduced from 1.0 for smoother response during transients)
        const double max_angle_velocity = 0.5;  // rad/s
        double angle_rate_limit = max_angle_velocity * dt;
        
        double dtheta = theta - this->last_roll_angle_;
        if (std::abs(dtheta) > angle_rate_limit)
        {
            theta = this->last_roll_angle_ + std::clamp(dtheta, -angle_rate_limit, angle_rate_limit);
        }
        
        double dphi = phi - this->last_pitch_angle_;
        if (std::abs(dphi) > angle_rate_limit)
        {
            phi = this->last_pitch_angle_ + std::clamp(dphi, -angle_rate_limit, angle_rate_limit);
        }
        
        // SMOOTHING FILTER: Apply exponential moving average (low-pass filter)
        // This smooths out high-frequency oscillation from IK solver during transients
        // while still allowing the gimbal to track commanded angles
        this->filtered_roll_angle_ = angle_filter_alpha_ * theta + (1.0 - angle_filter_alpha_) * this->filtered_roll_angle_;
        this->filtered_pitch_angle_ = angle_filter_alpha_ * phi + (1.0 - angle_filter_alpha_) * this->filtered_pitch_angle_;

        // Store computed angles; apply them in PostUpdate to avoid ordering conflicts
        this->last_roll_angle_ = this->filtered_roll_angle_;
        this->last_pitch_angle_ = this->filtered_pitch_angle_;
        
        // --- 4. Thrust ---
        // Thrust is handled externally by gz-sim-multicopter-motor-model-system, no action needed here.
    }

    // First-order system dynamics: dL/dt = (L_target - L) / tau
    void ApplyFirstOrderDynamics(double &current_len, double &velocity, 
                                  double target_len, double dt)
    {
        if (this->servo_time_constant_ > 0.0)
        {
            double error = target_len - current_len;
            double desired_dot = error / this->servo_time_constant_;
            
            // Clamp velocity to physical max speed
            double max_speed = this->servo_max_speed_; // m/s
            velocity = std::clamp(desired_dot, -max_speed, max_speed);
            
            current_len += velocity * dt;
            
            // STABILITY: If velocity is essentially zero and error is tiny, snap to target
            // to prevent oscillation from floating-point precision issues
            const double velocity_threshold = 1e-8;  // m/s
            const double error_threshold = 1e-8;     // meters
            if (std::abs(velocity) < velocity_threshold && std::abs(error) < error_threshold)
            {
                current_len = target_len;
                velocity = 0.0;
            }
        }
        else
        {
            // Fallback: pure rate limiter behavior (previous implementation)
            double max_change = this->servo_max_speed_ * dt;
            double error = target_len - current_len;
            current_len += std::clamp(error, -max_change, max_change);
            velocity = (current_len - target_len) / (dt > 0 ? dt : 1.0);
        }
        
        // Clamp length to physical limits
        current_len = std::clamp(current_len, this->servo_min_len_, this->servo_max_len_);
    }

    // Second-order system dynamics using RK2 integration
    // d²L/dt² + 2*zeta*wn*dL/dt + wn²*L = wn²*L_target
    void ApplySecondOrderDynamics(double &current_len, double &velocity, 
                                   double target_len, double dt)
    {
        double wn = this->servo_natural_freq_;   // Natural frequency (rad/s)
        double zeta = this->servo_damping_ratio_; // Damping ratio
        double max_speed = this->servo_max_speed_;
        
        // RK2 method for improved numerical stability
        // State: [L, v] where v = dL/dt
        // Derivatives: dL/dt = v, dv/dt = wn²(L_target - L) - 2*zeta*wn*v
        
        // Stage 1: compute initial derivatives
        double k1_L = velocity;
        double k1_v = wn * wn * (target_len - current_len) - 2.0 * zeta * wn * velocity;
        
        // Clamp velocity derivative to avoid numerical instability
        k1_v = std::clamp(k1_v, -max_speed * 10.0, max_speed * 10.0);
        
        // Stage 2: compute mid-point derivatives
        double mid_L = current_len + 0.5 * dt * k1_L;
        double mid_v = velocity + 0.5 * dt * k1_v;
        
        double k2_L = mid_v;
        double k2_v = wn * wn * (target_len - mid_L) - 2.0 * zeta * wn * mid_v;
        k2_v = std::clamp(k2_v, -max_speed * 10.0, max_speed * 10.0);
        
        // Update state using RK2 result
        current_len += dt * k2_L;
        velocity += dt * k2_v;
        
        // Clamp velocity to max speed
        velocity = std::clamp(velocity, -max_speed, max_speed);
        
        // STABILITY: If velocity is essentially zero and error is tiny, snap to target
        // to prevent oscillation from floating-point precision issues
        const double velocity_threshold = 1e-8;  // m/s
        const double error_threshold = 1e-8;     // meters
        double error = target_len - current_len;
        if (std::abs(velocity) < velocity_threshold && std::abs(error) < error_threshold)
        {
            current_len = target_len;
            velocity = 0.0;
        }
        
        // Clamp length to physical limits
        current_len = std::clamp(current_len, this->servo_min_len_, this->servo_max_len_);
    }

    // Callback for PX4 Servo 0 command (Roll control)
    void OnServo0Command(const msgs::Double &_msg)
    {
        // Extract normalized [-1, 1] input from PX4
        this->servo_0_command_ = _msg.data();
        
        // Convert input format to servo length target
        this->servo1_target_len_ = ConvertInputToServoLength(this->servo_0_command_, 0);
        
        gzmsg << "TVCPlugin::OnServo0Command - input: " << this->servo_0_command_ 
              << ", target_len: " << this->servo1_target_len_ << std::endl;
    }

    // Callback for PX4 Servo 1 command (Pitch control)
    void OnServo1Command(const msgs::Double &_msg)
    {
        // Extract normalized [-1, 1] input from PX4
        this->servo_1_command_ = _msg.data();
        
        // Convert input format to servo length target
        this->servo2_target_len_ = ConvertInputToServoLength(this->servo_1_command_, 1);
        
        gzmsg << "TVCPlugin::OnServo1Command - input: " << this->servo_1_command_ 
              << ", target_len: " << this->servo2_target_len_ << std::endl;
    }

    // Helper: Convert input value (format-specific) to servo length target
    // Uses real calibration parameters for accurate conversion
    // servo_index: 0 for servo1 (pitch), 1 for servo2 (roll)
    double ConvertInputToServoLength(double input_value, int servo_index)
    {
        double target_len = 0.0;
        double L_neutral, a_cal, p_neutral, L_min, L_max;
        
        // Get parameters for this servo
        if (servo_index == 0)  // Servo 1 (pitch)
        {
            a_cal = this->servo_calibration_.a3;
            p_neutral = this->servo_calibration_.p01;
            L_min = this->servo_calibration_.L3_min;
            L_max = this->servo_calibration_.L3_max;
        }
        else  // Servo 2 (roll)
        {
            a_cal = this->servo_calibration_.a6;
            p_neutral = this->servo_calibration_.p02;
            L_min = this->servo_calibration_.L6_min;
            L_max = this->servo_calibration_.L6_max;
        }
        
        // Compute neutral lengths from geometry
        double L3_neutral, L6_neutral;
        ComputeNeutralLengths(this->gimbal_geometry_, L3_neutral, L6_neutral);
        L_neutral = (servo_index == 0) ? L3_neutral : L6_neutral;
        
        switch (this->input_format_)
        {
            case InputFormat::NORMALIZED_MINUS1_1:
            {
                // PX4 normalized [-1, 1] format
                // Convert normalized to PWM range: PWM = p_neutral + input * 500 µs
                // (±1 maps to ±500 µs from neutral, standard RC convention)
                double pwm_value = p_neutral + input_value * 500.0;
                // Clamp PWM to valid range
                pwm_value = std::clamp(pwm_value, p_neutral - 500.0, p_neutral + 500.0);
                // Convert PWM to servo length using calibration formula:
                // L_target = L_neutral + a * (pwm - p_neutral)
                target_len = L_neutral + a_cal * (pwm_value - p_neutral);
                break;
            }
            case InputFormat::PWM_1000_2000:
            {
                // Traditional RC PWM [1000, 2000] format
                // L_target = L_neutral + a * (pwm - p_neutral)
                double pwm_value = input_value;
                pwm_value = std::clamp(pwm_value, 1000.0, 2000.0);
                target_len = L_neutral + a_cal * (pwm_value - p_neutral);
                break;
            }
            case InputFormat::SERVO_LENGTH:
            {
                // Direct servo length command (already in meters)
                target_len = input_value;
                break;
            }
            case InputFormat::JOINT_ANGLE:
            {
                // Direct joint angle command
                // This is more complex - would need forward kinematics to convert angle to length
                // For now, use a simple approximation: L = L_neutral + angle * scale_factor
                double angle_scale = (servo_index == 0) ? L3_neutral * 0.02 : L6_neutral * 0.02;
                target_len = L_neutral + input_value * angle_scale;
                break;
            }
            default:
            {
                // Fallback to normalized
                double pwm_value = p_neutral + input_value * 500.0;
                pwm_value = std::clamp(pwm_value, p_neutral - 500.0, p_neutral + 500.0);
                target_len = L_neutral + a_cal * (pwm_value - p_neutral);
                break;
            }
        }
        
        // Clamp to physical servo limits
        target_len = std::clamp(target_len, L_min, L_max);
        return target_len;
    }

    // Callback for legacy Gazebo Transport topic
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
        
        // Clamp the commanded length to the configured physical limits
        this->servo1_target_len_ = std::clamp(_msg.data(0), this->servo_min_len_, this->servo_max_len_);
        this->servo2_target_len_ = std::clamp(_msg.data(1), this->servo_min_len_, this->servo_max_len_);
    
          // Debug: print received (and clamped) targets so we can confirm the plugin
          // receives messages when running the simulator.
        gzmsg << "TVCPlugin::OnServoCommand - targets: "
            << this->servo1_target_len_ << ", " << this->servo2_target_len_ << std::endl;

    }

    // ISystemPostUpdate
    virtual void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override
    {
        // Publish computed angle setpoints to topics for JointPositionController to consume
        if (this->roll_angle_pub_)
        {
            msgs::Double roll_msg;
            roll_msg.set_data(this->last_roll_angle_);
            this->roll_angle_pub_->Publish(roll_msg);
        }
        
        if (this->pitch_angle_pub_)
        {
            msgs::Double pitch_msg;
            pitch_msg.set_data(this->last_pitch_angle_);
            this->pitch_angle_pub_->Publish(pitch_msg);
        }
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
    double servo_time_constant_; // seconds (tau) for first-order servo model
    double servo_natural_freq_; // rad/s for second-order system (natural frequency)
    double servo_damping_ratio_; // dimensionless for second-order system (zeta)
    std::string dynamics_model_; // "first_order" or "second_order"
    double servo_min_len_; // meters (configurable via SDF)
    double servo_max_len_; // meters (configurable via SDF)
    
    // --- State ---
    double servo1_current_len_; // The "real" simulated length
    double servo2_current_len_;
    double servo1_target_len_;  // The "commanded" length from Python
    double servo2_target_len_;
    double servo1_velocity_; // Velocity state for second-order dynamics
    double servo2_velocity_;
    // Last computed joint angles (set in PreUpdate, applied in PostUpdate)
    double last_roll_angle_ = 0.0;
    double last_pitch_angle_ = 0.0;
    // Filtered angles (exponential moving average low-pass filter)
    // Used to smooth out high-frequency oscillation during transients
    double filtered_roll_angle_ = 0.0;
    double filtered_pitch_angle_ = 0.0;
    static constexpr double angle_filter_alpha_ = 0.3;  // EMA filter coefficient (0.0-1.0)
                                                         // Lower = more smoothing, slower response
                                                         // Higher = less smoothing, faster response
    
    // --- Input Configuration (New) ---
    std::string input_mode_;  // "joint_position" or "transport_topic"
    InputFormat input_format_; // Format of input commands
    std::string servo_0_topic_; // Topic for Servo 0 (Roll)
    std::string servo_1_topic_; // Topic for Servo 1 (Pitch)
    std::string transport_topic_; // Legacy transport topic
    double servo_0_command_; // Last command from servo 0 topic
    double servo_1_command_; // Last command from servo 1 topic
    
    // --- Angle Output Topics (for JointPositionController) ---
    std::string roll_angle_topic_;  // Topic to publish roll angle setpoint
    std::string pitch_angle_topic_; // Topic to publish pitch angle setpoint
    std::unique_ptr<transport::Node::Publisher> roll_angle_pub_;
    std::unique_ptr<transport::Node::Publisher> pitch_angle_pub_;
    
    // --- Gimbal Geometry & Calibration (Real Kinematics) ---
    GimbalGeometry gimbal_geometry_;
    ServoCalibration servo_calibration_;
    
    // --- PX4 Transport Nodes ---
    std::unique_ptr<transport::Node> servo_0_node_;
    std::unique_ptr<transport::Node> servo_1_node_;
};

// Register the plugin
GZ_ADD_PLUGIN(TVCPlugin, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)

//
// Created by joshua on 7/13/17.
//
#include "sim_env/Controller.h"
#include "sim_env/utils/EigenUtils.h"
#include <boost/math/constants/constants.hpp>
#include <cmath>

using namespace sim_env;

sim_env::PIDController::PIDController(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    reset();
}

sim_env::PIDController::~PIDController()
{
    // nothing to do here
}

void sim_env::PIDController::setKp(float kp)
{
    _kp = kp;
}

void sim_env::PIDController::setKi(float ki)
{
    _ki = ki;
}

void sim_env::PIDController::setKd(float kd)
{
    _kd = kd;
}

void sim_env::PIDController::setGains(float kp, float ki, float kd)
{
    setKp(kp);
    setKi(ki);
    setKd(kd);
}

void sim_env::PIDController::setTarget(float target_state)
{
    if (target_state != _target) {
        reset();
        _target = target_state;
    }
}

float sim_env::PIDController::getTarget() const
{
    return _target;
}

bool sim_env::PIDController::isTargetSatisfied(float current_state, float threshold) const
{
    return std::fabs(current_state - _target) < threshold;
}

float sim_env::PIDController::control(float current_state)
{
    float error = _target - current_state;
    float delta_error = 0.0f;
    if (not std::isnan(_prev_error)) {
        delta_error = error - _prev_error;
    }
    _prev_error = error;
    _integral_part += error;
    float output = _kp * error + _ki * _integral_part + _kd * delta_error;
    //    if (isTargetSatisfied(current_state, 0.001f)) {
    //        auto logger = DefaultLogger::getInstance();
    //        std::stringstream ss;
    //        ss << "target is satisfied. output is " << output;
    //        ss << "error is " << error << " current state is " << current_state;
    //        ss << "kp: " << _kp << " ki" << _ki << "kd" << _kd;
    //        logger->logDebug(ss.str());
    //    }
    return output;
}

void sim_env::PIDController::reset()
{
    _integral_part = 0.0;
    _prev_error = nanf("");
}

////////////////////// IndependentMDPIDController //////////////////////////////
IndependentMDPIDController::IndependentMDPIDController(float kp, float ki, float kd)
{
    _default_ki = ki;
    _default_kp = kp;
    _default_kd = kd;
    setGains(kp, ki, kd);
}

IndependentMDPIDController::~IndependentMDPIDController()
{
    // nothing to do here.
}

void IndependentMDPIDController::setTarget(const Eigen::VectorXf& target_state)
{
    if (target_state.size() != _controllers.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setTarget]"
                                 "Invalid input state dimension.");
    }
    for (size_t i = 0; i < target_state.size(); ++i) {
        _controllers.at(i).setTarget(target_state[i]);
    }
}

void IndependentMDPIDController::getTarget(Eigen::VectorXf& target_state) const
{
    target_state.resize(_controllers.size());
    for (size_t i = 0; i < _controllers.size(); ++i) {
        target_state[i] = _controllers.at(i).getTarget();
    }
}

void IndependentMDPIDController::control(Eigen::VectorXf& output,
    const Eigen::VectorXf& current_state)
{
    if (_controllers.size() != current_state.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::control]"
                                 "Invalid input state dimension.");
    }
    output.resize(_controllers.size());
    for (size_t i = 0; i < _controllers.size(); ++i) {
        output[i] = _controllers.at(i).control(current_state[i]);
    }
}

void IndependentMDPIDController::reset()
{
    for (auto& controller : _controllers) {
        controller.reset();
    }
}

bool IndependentMDPIDController::isTargetSatisfied(Eigen::VectorXf& current_state, float threshold) const
{
    bool target_satisfied = true;
    for (size_t i = 0; i < _controllers.size(); ++i) {
        target_satisfied = target_satisfied and _controllers.at(i).isTargetSatisfied(current_state[i], threshold);
    }
    return target_satisfied;
}

void IndependentMDPIDController::setGains(float kp, float ki, float kd)
{
    for (auto& controller : _controllers) {
        controller.setGains(kp, ki, kd);
    }
}

void IndependentMDPIDController::setGains(const Eigen::VectorXf& kps, const Eigen::VectorXf& kis, const Eigen::VectorXf& kds)
{
    if (_controllers.size() != kps.size() or _controllers.size() != kis.size() or _controllers.size() != kds.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setGains]"
                                 "Invalid input vector dimension.");
    }
    for (size_t i = 0; i < _controllers.size(); ++i) {
        _controllers.at(i).setGains(kps[i], kis[i], kds[i]);
    }
}

void IndependentMDPIDController::setKps(const Eigen::VectorXf& kps)
{
    if (_controllers.size() != kps.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setKps]"
                                 "Invalid input vector dimension.");
    }
    for (size_t i = 0; i < _controllers.size(); ++i) {
        _controllers.at(i).setKp(kps[i]);
    }
}

void IndependentMDPIDController::setKp(float kp)
{
    for (auto& controller : _controllers) {
        controller.setKp(kp);
    }
}

void IndependentMDPIDController::setKis(const Eigen::VectorXf& kis)
{
    if (_controllers.size() != kis.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setKis]"
                                 "Invalid input vector dimension.");
    }
    for (size_t i = 0; i < _controllers.size(); ++i) {
        _controllers.at(i).setKi(kis[i]);
    }
}

void IndependentMDPIDController::setKi(float ki)
{
    for (auto& controller : _controllers) {
        controller.setKi(ki);
    }
}

void IndependentMDPIDController::setKds(const Eigen::VectorXf& kds)
{
    if (_controllers.size() != kds.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setKds]"
                                 "Invalid input vector dimension.");
    }
    for (size_t i = 0; i < _controllers.size(); ++i) {
        _controllers.at(i).setKd(kds[i]);
    }
}

void IndependentMDPIDController::setKd(float kd)
{
    for (auto& controller : _controllers) {
        controller.setKd(kd);
    }
}

unsigned int IndependentMDPIDController::getStateDimension()
{
    return (unsigned int)_controllers.size();
}

void IndependentMDPIDController::setStateDimension(unsigned int dim)
{
    if (dim != _controllers.size()) {
        unsigned int prev_dim = (unsigned int)_controllers.size();
        _controllers.resize(dim);
        reset();
        for (unsigned int i = prev_dim; i < _controllers.size(); ++i) {
            _controllers.at(i).setGains(_default_kp, _default_ki, _default_kd);
        }
    }
}

//*************************** RobotController ************************************//
RobotController::~RobotController() = default;

//*************************** RobotPositionController ****************************//
RobotPositionController::RobotPositionController(RobotPtr robot,
    RobotVelocityControllerPtr velocity_controller)
    : _pid_controller(1.0, 0.0, 0.0)
    , _velocity_controller(velocity_controller)
    , _robot(robot)
{
}

RobotPositionController::~RobotPositionController()
{
}

void RobotPositionController::setPositionProjectionFn(PositionProjectionFn pos_constraint)
{
    _pos_proj_fn = pos_constraint;
}

void RobotPositionController::setVelocityProjectionFn(VelocityProjectionFn vel_constraint)
{
    _vel_proj_fn = vel_constraint;
}

void RobotPositionController::setTarget(const Eigen::VectorXf& position)
{
    setTargetPosition(position);
}

void RobotPositionController::setTargetPosition(const Eigen::VectorXf& position)
{
    if (_robot.expired()) {
        LoggerPtr logger = DefaultLogger::getInstance();
        logger->logErr("Can not access underlying robot; the pointer is not valid anymore!",
            "[sim_env::RobotPositionController::setTargetPosition]");
    }
    RobotPtr robot = _robot.lock();
    LoggerPtr logger = robot->getWorld()->getLogger();
    // std::stringstream ss;
    // ss << "Setting target position " << position.transpose();
    // logger->logDebug(ss.str(), "[sim_env::RobotPositionController::setTargetPosition]");
    _pid_controller.setStateDimension((unsigned int)position.size());
    _pid_controller.setTarget(position);
}

unsigned int RobotPositionController::getTargetDimension() const
{
    auto robot = _robot.lock();
    return robot->getNumActiveDOFs();
}

RobotPtr RobotPositionController::getRobot() const
{
    return _robot.lock();
}

inline float cyclicPositionError(const Eigen::Array2f& pos_range, float pos, float target)
{
    float error = target - pos;
    float overflow_error = pos_range[1] - pos + target - pos_range[0];
    float underflow_error = pos_range[0] - pos + target - pos_range[1];
    error = std::abs(error) < std::abs(overflow_error) ? error : overflow_error;
    error = std::abs(error) < std::abs(underflow_error) ? error : underflow_error;
    return error;
}

bool RobotPositionController::control(const Eigen::VectorXf& positions, const Eigen::VectorXf& velocities,
    float timestep, RobotConstPtr robot,
    Eigen::VectorXf& output)
{
    Eigen::VectorXf target_position;
    _pid_controller.getTarget(target_position);
    if (target_position.size() != robot->getActiveDOFs().size()) {
        LoggerPtr logger = robot->getWorld()->getLogger();
        logger->logErr("The provided target position has different dimension from the active DOFs."
                       "[sim_env::RobotPositionController::setTargetPosition]");
        std::stringstream ss;
        ss << "target size is " << target_position.size() << " active dof size is " << robot->getNumActiveDOFs();
        logger->logErr(ss.str());
        return false;
    }
    // project target position onto constraint set (this should only do anything if the user set invalid target positions)
    if (_pos_proj_fn) {
        _pos_proj_fn(target_position, robot);
    }
    Eigen::VectorXf target_velocities(positions.size());
    //    _pid_controller.control(target_velocities, positions);
    // TODO see whether we still can use a PID somehow
    // TODO this is not moving in a straight line. we could make the decision on
    // TODO whether to move in a straight line or not dependent on a parameter
    // Eigen::ArrayX2f velocity_limits = robot->getDOFVelocityLimits();
    // Eigen::ArrayX2f acceleration_limits = robot->getDOFAccelerationLimits();
    // Eigen::VectorXf delta_position = target_position - positions;
    Eigen::VectorXi dof_indices = robot->getActiveDOFs();
    assert(dof_indices.size() == positions.size());
    DOFInformation dof_info;
    for (int idx = 0; idx < dof_indices.size(); ++idx) {
        // get dof information
        robot->getDOFInformation(dof_indices[idx], dof_info);
        float delta_position = target_position[idx] - positions[idx];
        if (dof_info.cyclic) {
            delta_position = cyclicPositionError(dof_info.position_limits, positions[idx], target_position[idx]);
        }
        // first, command max velocity for each dof separately
        float velocity_sign(1.0f);
        if (std::signbit(delta_position)) {
            // need negative velocity
            target_velocities[idx] = dof_info.velocity_limits[0];
            assert(std::signbit(target_velocities[idx]));
            velocity_sign = -1.0f;
        } else {
            // need positive velocity
            target_velocities[idx] = dof_info.velocity_limits[1];
            velocity_sign = 1.0f;
        }
        // next compute the maximum velocity we can have to not overshoot
        float abs_position_error = std::abs(delta_position);
        float abs_max_break_accel = 0.0f;
        if (std::signbit(velocities[idx])) {
            abs_max_break_accel = std::abs(dof_info.acceleration_limits[1]);
        } else {
            abs_max_break_accel = std::abs(dof_info.acceleration_limits[0]);
        }
        float abs_max_break_velocity = std::sqrt(2.0f * abs_position_error * abs_max_break_accel);
        // finally, set the target velocity such maximal, but such that we do not overshoot.
        target_velocities[idx] = velocity_sign * std::min(abs_max_break_velocity, std::abs(target_velocities[idx]));
    }
    // { // TODO delete this block

    //     auto robot = _robot.lock();
    //     auto world = robot->getWorld();
    //     auto logger = world->getLogger();
    //     logger->logDebug(boost::format("Position error: x:%1%, y: %2%, theta: %3%") % delta_position[0] % delta_position[1] % delta_position[2], "[sim_env::PositionController]");
    // }
    if (_vel_proj_fn) {
        _vel_proj_fn(target_velocities, robot);
    }
    _velocity_controller->setTargetVelocity(target_velocities);
    _velocity_controller->control(positions, velocities, timestep, robot, output);
    return true;
}

///////////////////////////// RobotVelocityController ///////////////////////////////
//RobotVelocityController::RobotVelocityController(RobotPtr robot):
//        _pid_controller(10.0, 0.0, 0.0),
//        _robot(robot) {
//}

RobotVelocityController::~RobotVelocityController() = default;

void RobotVelocityController::setTarget(const Eigen::VectorXf& target)
{
    setTargetVelocity(target);
}

//void RobotVelocityController::setTargetVelocity(const Eigen::VectorXf &velocity) {
//    if (_robot.expired()) {
//        LoggerPtr logger = DefaultLogger::getInstance();
//        logger->logErr("Can not access underlying robot; the pointer is not valid anymore!",
//                       "[sim_env::RobotVelocityController::setTargetVelocity]");
//    }
//    RobotPtr robot = _robot.lock();
//    Eigen::VectorXf new_target = velocity;
//    LoggerPtr logger = robot->getWorld()->getLogger();
//    // scale the target velocity to limits
//    using namespace utils::eigen;
//    ScalingResult scaling_result = scaleToLimits(new_target, robot->getDOFVelocityLimits());
//    if (scaling_result == ScalingResult::Failure) {
//        logger->logWarn("Impossible target velocity direction detected. The requested velocity can not be scaled to the given limits",
//                        "[sim_env::RobotVelocityController::setTargetVelocity]");
//    } else if (scaling_result == ScalingResult::Scaled) {
//        logger->logWarn("Requested velocity is out of limits. Scaling it down.",
//                        "[sim_env::RobotVelocityController::setTargetVelocity]");
//    }
//    std::stringstream ss;
//    ss << "Setting target velocity " << new_target.transpose();
//    logger->logDebug(ss.str(), "[sim_env::RobotVelocityController::setTargetVelocity]");
//    // set the new target
//    _pid_controller.setStateDimension((unsigned int) new_target.size());
//    _pid_controller.setTarget(new_target);
//}
//
//bool RobotVelocityController::control(const Eigen::VectorXf &positions, const Eigen::VectorXf &velocities,
//                                      float timestep, RobotConstPtr robot,
//                                      Eigen::VectorXf &output) {
//    Eigen::VectorXf target_velocity;
//    _pid_controller.getTarget(target_velocity);
//    if (target_velocity.size() != robot->getActiveDOFs().size()) {
//        LoggerPtr logger = robot->getWorld()->getLogger();
//        logger->logErr("The provided target position has different dimension from the active DOFs."
//                               "[sim_env::RobotVelocityController::setTargetVelocity]");
//        return false;
//    }
//    _pid_controller.control(output, velocities);
////    LoggerPtr logger = robot->getWorld()->getLogger();
////    std::stringstream ss;
////    ss << "Target velocities are " << target_velocity.transpose();
////    ss << " Current velocities are " << velocities.transpose();
////    logger->logDebug(ss.str());
//    // TODO limit efforts?
//    return true;
//}
//
//IndependentMDPIDController& RobotVelocityController::getPIDController() {
//   return _pid_controller;
//}
//
//LoggerPtr RobotVelocityController::getLogger() {
//    if (_robot.expired()) {
//        return DefaultLogger::getInstance();
//    }
//    RobotPtr robot = _robot.lock();
//    return robot->getWorld()->getLogger();
//}

SE2RobotPositionController::SE2RobotPositionController(RobotPtr robot, RobotVelocityControllerPtr velocity_controller)
    : _robot(robot)
    , _velocity_controller(velocity_controller)
{
    assert(robot->getNumDOFs() >= 3);
    assert(!robot->isStatic());
    // velocity limits
    auto dof_vel_limits = robot->getDOFVelocityLimits();
    float x_limit = std::min(std::abs(dof_vel_limits(0, 0)), std::abs(dof_vel_limits(0, 1)));
    float y_limit = std::min(std::abs(dof_vel_limits(1, 0)), std::abs(dof_vel_limits(1, 1)));
    _cartesian_vel_limit = std::min(x_limit, y_limit);
    _angular_vel_limit = std::min(std::abs(dof_vel_limits(2, 0)), std::abs(dof_vel_limits(2, 1)));
    // accelaration limits
    auto dof_acc_limits = robot->getDOFAccelerationLimits();
    x_limit = std::min(std::abs(dof_acc_limits(0, 0)), std::abs(dof_acc_limits(0, 1)));
    y_limit = std::min(std::abs(dof_acc_limits(1, 0)), std::abs(dof_acc_limits(1, 1)));
    _cartesian_acc_limit = std::min(x_limit, y_limit);
    _angular_acc_limit = std::min(std::abs(dof_acc_limits(2, 0)), std::abs(dof_acc_limits(2, 1)));
    _set_point.resize(3);
}

SE2RobotPositionController::~SE2RobotPositionController()
{
}

void SE2RobotPositionController::setPositionProjectionFn(PositionProjectionFn pos_constraint)
{
    _pos_proj_fn = pos_constraint;
}

void SE2RobotPositionController::setVelocityProjectionFn(VelocityProjectionFn vel_constraint)
{
    _vel_proj_fn = vel_constraint;
}

inline float normlizeOrientation(float v)
{
    const float pi = boost::math::constants::pi<float>();
    float sign = std::signbit(v) ? -1.0f : 1.0f;
    return sign * (std::abs(v) - std::floor((std::abs(v) + pi) / (2.0f * pi)) * 2.0f * pi);
}

inline float shortestSO2Direction(float val_1, float val_2)
{
    float value = val_2 - val_1;
    if (std::abs(value) > boost::math::constants::pi<float>()) {
        if (value > 0.0f) { // val_2 > val_1
            value -= 2.0f * boost::math::constants::pi<float>();
        } else {
            value += 2.0f * boost::math::constants::pi<float>();
        }
    }
    return value;
}

void SE2RobotPositionController::setTarget(const Eigen::VectorXf& target)
{
    static const std::string log_prefix("[SE2RobotPositionController::setTarget]");
    if (target.size() != 3) {
        auto robot = getRobot();
        auto logger = robot->getWorld()->getLogger();
        logger->logErr(boost::format("Target has invalid dimension. Expected dimension is 3, actual dimension is %1%") % target.size(), log_prefix);
        return;
    }
    _last_target = target;
    // normalize target orientation
    _last_target[2] = normlizeOrientation(_last_target[2]);

    // logger->logDebug(boost::format("Target %1%, %2%, %3%") % _last_target[0] % _last_target[1] % _last_target[2], log_prefix);
}

void SE2RobotPositionController::setTargetPosition(const Eigen::VectorXf& position)
{
    setTarget(position);
}

unsigned int SE2RobotPositionController::getTargetDimension() const
{
    return 3;
}

RobotPtr SE2RobotPositionController::getRobot() const
{
    auto robot = _robot.lock();
    if (!robot) {
        std::logic_error("[SE2RobotPositionController::getRobot] Could not acquire robot. Weak pointer expired.");
    }
    return robot;
}

bool SE2RobotPositionController::control(const Eigen::VectorXf& positions,
    const Eigen::VectorXf& velocities,
    float timestep,
    RobotConstPtr robot,
    Eigen::VectorXf& output)
{
    static const std::string log_prefix("[SE2RobotPositionController::control]");
    auto logger = robot->getConstWorld()->getConstLogger();
    if (_last_target.size() != 3) {
        logger->logWarn("No target set. Can not control any position", log_prefix);
        return false;
    }
    assert(positions.size() >= 3);
    // compute error in cartesian position
    Eigen::Vector2f cart_error = _last_target.head(2) - positions.head(2);
    // logger->logDebug(boost::format("Target %1%, %2%, %3%") % _last_target[0] % _last_target[1] % _last_target[2], log_prefix);
    float cart_error_norm = cart_error.norm();
    if (cart_error_norm > 0.0f) {
        cart_error /= cart_error_norm;
    }
    // compute the maximum velocity we can have to not overshoot
    float abs_max_break_velocity = std::sqrt(2.0f * cart_error_norm * _cartesian_acc_limit);
    float cart_vel = std::min(abs_max_break_velocity, _cartesian_vel_limit);
    // do the same for angular velocity
    // float angular_error = _last_target[2] - positions[2];
    float angular_error = shortestSO2Direction(positions[2], _last_target[2]);
    float angular_error_norm = std::abs(angular_error);
    float max_break_velocity_angular = std::sqrt(2.0f * angular_error_norm * _angular_acc_limit);
    float omega = std::min(max_break_velocity_angular, _angular_vel_limit);
    // if we are at our target position, command zero velocity, else compute a velocity taking us to our target
    if (angular_error_norm < 1e-4 and cart_error_norm < 1e-4) {
        _set_point.setZero();
    } else {
        // debug log
        // logger->logDebug(boost::format("cart_error: %1%, cart_vel: %2%, angular_error: %3%, angular_vel: %4%") % cart_error_norm % cart_vel % angular_error % omega, log_prefix);
        // now compute how much time we will need for each to reach its destination
        // we travel for some time at max velocity, followed by a decelaration phase
        // in the decelaration we slow down from v to 0, i.e. its duration is v / a. The distance we travel
        // in the decelaration is 0.5f * v / a. Accordingly, the time we are at the max velocity is t = s / v - 0.5 * v / a.
        // If we are in the deceleration phase already, t becomes negative.
        float t_angular_plateau = 0.0f;
        if (angular_error_norm > 0.0f) {
            t_angular_plateau = angular_error_norm / omega - 0.5f * omega / _angular_acc_limit;
        }
        float t_cart_plateau = 0.0f;
        if (cart_error_norm > 0.0f) {
            t_cart_plateau = cart_error_norm / cart_vel - 0.5f * cart_vel / _cartesian_acc_limit;
        }
        float t_angular = std::max(t_angular_plateau, 0.0f) + omega / _angular_acc_limit;
        float t_cart = std::max(t_cart_plateau, 0.0f) + cart_vel / _cartesian_acc_limit;
        // logger->logDebug(boost::format("Estimated duration till target, cart: %1%, angular %2%") % t_cart % t_angular, log_prefix);
        if (t_cart < t_angular) { // we will take more time to reach our angle destination, slow down cartesian
            float a = _cartesian_acc_limit * t_angular;
            float b = _cartesian_acc_limit * _cartesian_acc_limit * t_angular * t_angular - 2.0f * _cartesian_acc_limit * cart_error_norm;
            assert(b >= 0.0f);
            cart_vel = a - std::sqrt(b);
        } else { // we will take more time to reach our cartesian destination, slow down angular
            float a = _angular_acc_limit * t_cart;
            float b = _angular_acc_limit * _angular_acc_limit * t_cart * t_cart - 2.0f * _angular_acc_limit * angular_error_norm;
            assert(b >= 0.0f);
            omega = a - std::sqrt(b);
        }
        // logger->logDebug(boost::format("Commanded velocities: cart: %1%, angular: %2%") % cart_vel % omega, log_prefix);
        // logger->logDebug(boost::format("Cartesian velocity direction (x, y): %1%, %2%") % cart_error[0] % cart_error[1], log_prefix);
        _set_point.head(2) = cart_vel * cart_error;
        _set_point[2] = omega * angular_error / angular_error_norm;
        // TODO We should also define the velocities during acceleration phase. For large position errors and max velocities,
        // TODO it takes the robot different amount of time to accelerate to the respective target velocity for each DoF.
        // TODO Hence, the robot will not move all the time in the direction that we command (this depends on the underlying velocity controller of course).
        // TODO Alternatively, could define a velocity controller that operates in the same way as this position controller, i.e.
        // TODO one that ensures that a commanded target velocity is reached synchronously for all DOFs.
    }
    _velocity_controller->setTargetVelocity(_set_point);
    _velocity_controller->control(positions, velocities, timestep, robot, output);
    return true;
}
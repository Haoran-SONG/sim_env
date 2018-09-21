//
// Created by joshua on 7/13/17.
//
#include "sim_env/Controller.h"
#include "sim_env/utils/EigenUtils.h"
#include <cmath>

using namespace sim_env;

sim_env::PIDController::PIDController(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    reset();
}

sim_env::PIDController::~PIDController() {
    // nothing to do here
}

void sim_env::PIDController::setKp(float kp) {
    _kp = kp;
}

void sim_env::PIDController::setKi(float ki) {
    _ki = ki;
}

void sim_env::PIDController::setKd(float kd) {
    _kd = kd;
}

void sim_env::PIDController::setGains(float kp, float ki, float kd) {
    setKp(kp);
    setKi(ki);
    setKd(kd);
}

void sim_env::PIDController::setTarget(float target_state) {
    if (target_state != _target) {
        reset();
        _target = target_state;
    }
}

float sim_env::PIDController::getTarget() const {
    return _target;
}

bool sim_env::PIDController::isTargetSatisfied(float current_state, float threshold) const {
    return std::fabs(current_state - _target) < threshold;
}

float sim_env::PIDController::control(float current_state) {
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

void sim_env::PIDController::reset() {
    _integral_part = 0.0;
    _prev_error = nanf("");
}

////////////////////// IndependentMDPIDController //////////////////////////////
IndependentMDPIDController::IndependentMDPIDController(float kp, float ki, float kd) {
    _default_ki = ki;
    _default_kp = kp;
    _default_kd = kd;
    setGains(kp, ki, kd);
}

IndependentMDPIDController::~IndependentMDPIDController() {
    // nothing to do here.
}

void IndependentMDPIDController::setTarget(const Eigen::VectorXf &target_state) {
    if (target_state.size() != _controllers.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setTarget]"
                                 "Invalid input state dimension.");
    }
    for (size_t i = 0; i < target_state.size(); ++i) {
        _controllers.at(i).setTarget(target_state[i]);
    }
}

void IndependentMDPIDController::getTarget(Eigen::VectorXf& target_state) const {
    target_state.resize(_controllers.size());
    for (size_t i = 0; i < _controllers.size(); ++i) {
        target_state[i] = _controllers.at(i).getTarget();
    }
}

void IndependentMDPIDController::control(Eigen::VectorXf &output,
                                         const Eigen::VectorXf &current_state) {
    if (_controllers.size() != current_state.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::control]"
                                 "Invalid input state dimension.");
    }
    output.resize(_controllers.size());
    for (size_t i = 0; i < _controllers.size(); ++i) {
        output[i] = _controllers.at(i).control(current_state[i]);
    }
}

void IndependentMDPIDController::reset() {
    for (auto& controller : _controllers) {
        controller.reset();
    }
}

bool IndependentMDPIDController::isTargetSatisfied(Eigen::VectorXf& current_state, float threshold) const {
    bool target_satisfied = true;
    for (size_t i = 0; i < _controllers.size(); ++i) {
        target_satisfied = target_satisfied and _controllers.at(i).isTargetSatisfied(current_state[i], threshold);
    }
    return target_satisfied;
}

void IndependentMDPIDController::setGains(float kp, float ki, float kd) {
    for (auto& controller : _controllers) {
        controller.setGains(kp, ki, kd);
    }
}

void IndependentMDPIDController::setGains(const Eigen::VectorXf& kps, const Eigen::VectorXf& kis, const Eigen::VectorXf& kds) {
    if (_controllers.size() != kps.size() or _controllers.size() != kis.size() or _controllers.size() != kds.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setGains]"
                                         "Invalid input vector dimension.");
    }
    for (size_t i = 0; i < _controllers.size(); ++i) {
        _controllers.at(i).setGains(kps[i], kis[i], kds[i]);
    }
}

void IndependentMDPIDController::setKps(const Eigen::VectorXf& kps) {
    if (_controllers.size() != kps.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setKps]"
                                         "Invalid input vector dimension.");
    }
    for (size_t i = 0; i < _controllers.size(); ++i) {
        _controllers.at(i).setKp(kps[i]);
    }
}

void IndependentMDPIDController::setKp(float kp) {
    for (auto& controller : _controllers) {
        controller.setKp(kp);
    }
}

void IndependentMDPIDController::setKis(const Eigen::VectorXf& kis) {
    if (_controllers.size() != kis.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setKis]"
                                         "Invalid input vector dimension.");
    }
    for (size_t i = 0; i < _controllers.size(); ++i) {
        _controllers.at(i).setKi(kis[i]);
    }
}

void IndependentMDPIDController::setKi(float ki) {
    for (auto& controller : _controllers) {
        controller.setKi(ki);
    }
}

void IndependentMDPIDController::setKds(const Eigen::VectorXf& kds) {
    if (_controllers.size() != kds.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setKds]"
                                         "Invalid input vector dimension.");
    }
    for (size_t i = 0; i < _controllers.size(); ++i) {
        _controllers.at(i).setKd(kds[i]);
    }
}

void IndependentMDPIDController::setKd(float kd) {
    for (auto& controller : _controllers) {
        controller.setKd(kd);
    }
}

unsigned int IndependentMDPIDController::getStateDimension() {
    return (unsigned int) _controllers.size();
}

void IndependentMDPIDController::setStateDimension(unsigned int dim) {
    if (dim != _controllers.size()) {
        unsigned int prev_dim = (unsigned int) _controllers.size();
        _controllers.resize(dim);
        reset();
        for (unsigned int i = prev_dim; i < _controllers.size(); ++i) {
            _controllers.at(i).setGains(_default_kp, _default_ki, _default_kd);
        }
    }
}

///////////////////////////// RobotPositionController ///////////////////////////////
RobotPositionController::RobotPositionController(RobotPtr robot,
                                                 RobotVelocityControllerPtr velocity_controller):
        _pid_controller(1.0, 0.0, 0.0),
        _velocity_controller(velocity_controller),
        _robot(robot) {
}

RobotPositionController::~RobotPositionController() {
}

void RobotPositionController::setTargetPosition(const Eigen::VectorXf& position) {
    if (_robot.expired()) {
        LoggerPtr logger = DefaultLogger::getInstance();
        logger->logErr("Can not access underlying robot; the pointer is not valid anymore!",
                       "[sim_env::RobotPositionController::setTargetPosition]");
    }
    RobotPtr robot = _robot.lock();
    LoggerPtr logger = robot->getWorld()->getLogger();
    std::stringstream ss;
    ss << "Setting target position " << position.transpose();
    logger->logDebug(ss.str(), "[sim_env::RobotPositionController::setTargetPosition]");
    _pid_controller.setStateDimension((unsigned int) position.size());
    _pid_controller.setTarget(position);
}

RobotPtr RobotPositionController::getRobot() const {
    return _robot.lock();
}

bool RobotPositionController::control(const Eigen::VectorXf &positions, const Eigen::VectorXf &velocities,
                                      float timestep, RobotConstPtr robot,
                                      Eigen::VectorXf &output) {
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
    Eigen::VectorXf target_velocities(positions.size());
//    _pid_controller.control(target_velocities, positions);
    // TODO see whether we still can use a PID somehow
    // TODO this is not moving in a straight line. we could make the decision on
    // TODO whether to move in a straight line or not dependent on a parameter
    Eigen::ArrayX2f velocity_limits = robot->getDOFVelocityLimits();
    Eigen::ArrayX2f acceleration_limits = robot->getDOFAccelerationLimits();
    Eigen::VectorXf delta_position = target_position - positions;
    for (int idx = 0; idx < delta_position.size(); ++idx) {
        // first, command max velocity for each dof separately
        float velocity_sign(1.0f);
        if (std::signbit(delta_position[idx])) {
            // need negative velocity
            target_velocities[idx] = velocity_limits(idx, 0);
            assert(std::signbit(target_velocities[idx]));
            velocity_sign = -1.0f;
        } else {
            // need positive velocity
            target_velocities[idx] = velocity_limits(idx, 1);
            velocity_sign = 1.0f;
        }
        // next compute the maximum velocity we can have to not overshoot
        float abs_position_error = std::abs(delta_position[idx]);
        float abs_max_break_accel = 0.0f;
        if (std::signbit(velocities[idx])) {
            abs_max_break_accel = std::abs(acceleration_limits(idx, 1));
        } else {
            abs_max_break_accel = std::abs(acceleration_limits(idx, 0));
        }
        float abs_max_break_velocity = std::sqrt(2.0f * abs_position_error * abs_max_break_accel);
        // finally, set the target velocity such maximal, but such that we do not overshoot.
        target_velocities[idx] = velocity_sign * std::min(abs_max_break_velocity, std::abs(target_velocities[idx]));
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

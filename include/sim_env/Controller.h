//
// Created by joshua on 7/13/17.
//

#ifndef SIM_ENV_CONTROLLER_H
#define SIM_ENV_CONTROLLER_H

#include <memory>
#include <Eigen/Dense>
#include <sim_env/SimEnv.h>

namespace sim_env {
    class Controller;
    typedef std::shared_ptr<Controller> ControllerPtr;
    typedef std::shared_ptr<const Controller> ControllerConstPtr;
    typedef std::weak_ptr<Controller> ControllerWeakPtr;
    typedef std::weak_ptr<const Controller> ControllerWeakConstPtr;

    class MDController;
    typedef std::shared_ptr<MDController> MDControllerPtr;
    typedef std::shared_ptr<const MDController> MDControllerConstPtr;
    typedef std::weak_ptr<MDController> MDControllerWeakPtr;
    typedef std::weak_ptr<const MDController> MDControllerWeakConstPtr;

    class PIDController;
    typedef std::shared_ptr<PIDController> PIDControllerPtr;
    typedef std::shared_ptr<const PIDController> PIDControllerConstPtr;
    typedef std::weak_ptr<PIDController> PIDControllerWeakPtr;
    typedef std::weak_ptr<const PIDController> PIDControllerWeakConstPtr;

    class IndependentMDPIDController;
    typedef std::shared_ptr<IndependentMDPIDController> IndependentMDPIDControllerPtr;
    typedef std::shared_ptr<const IndependentMDPIDController> IndependentMDPIDControllerConstPtr;
    typedef std::weak_ptr<IndependentMDPIDController> IndependentMDPIDControllerWeakPtr;
    typedef std::weak_ptr<const IndependentMDPIDController> IndependentMDPIDControllerWeakConstPtr;

    class RobotPositionController;
    typedef std::shared_ptr<RobotPositionController> RobotPositionControllerPtr;
    typedef std::shared_ptr<const RobotPositionController> RobotPositionControllerConstPtr;
    typedef std::weak_ptr<RobotPositionController> RobotPositionControllerWeakPtr;
    typedef std::weak_ptr<const RobotPositionController> RobotPositionControllerWeakConstPtr;

    class RobotVelocityController;
    typedef std::shared_ptr<RobotVelocityController> RobotVelocityControllerPtr;
    typedef std::shared_ptr<const RobotVelocityController> RobotVelocityControllerConstPtr;
    typedef std::weak_ptr<RobotVelocityController> RobotVelocityControllerWeakPtr;
    typedef std::weak_ptr<const RobotVelocityController> RobotVelocityControllerWeakConstPtr;

    /**
     * Base-class for controllers for one-dimensional states.
     */
    class Controller {
    public:
        /**
         * Set the current target state.
         * @param target_state - floating number target state
         */
        virtual void setTarget(float target_state) = 0;
        /**
         * Returns the last set target state.
         * @return last set target state
         */
        virtual float getTarget() const = 0;
        /**
         * Returns whether the target is reached for the given state.
         * @param current_state - current state of the system
         * @param threshold - error threshold
         * @return true, iff ||current_state - target_state|| < threshold according
         *          to an implementation specific norm ||.||
         */
        virtual bool isTargetSatisfied(float current_state, float threshold=0.001f) const = 0;
        /**
         * Compute control output for the current state.
         * @param current_state - state for which a control output should be computed
         * @return control signal
         */
        virtual float control(float current_state) = 0;
        /**
         * Reset the controller to its initial state.
         */
        virtual void reset() = 0;
    };

    /**
     * Base-class for multi-dimensional controllers.
     */
    class MDController {
    public:
        /**
         * Set the target state for the control.
         * @param target_state - target state of currently set dimension. Undefined behavior,
         *              the dimension differs from the last set dimension.
         */
        virtual void setTarget(const Eigen::VectorXf& target_state) = 0;

        /**
         * Returns the last set target state.
         * @param target_state - will contain the last set target state
         */
        virtual void getTarget(Eigen::VectorXf& target_state) const = 0;

        /**
         * Returns whether the target is reached for the given state.
         * @param current_state - current state of the system
         * @param threshold - error threshold
         * @return true, iff ||current_state - target_state|| < threshold according
         *          to an implementation specific norm ||.||
         */
        virtual bool isTargetSatisfied(Eigen::VectorXf& current_state, float threshold=0.001f) const = 0;

        /**
         * Compute control output for the current state.
         * @param output - the computed control signal
         * @param current_state - the state to compute a control for.
         *                        Must have dimension equal to getStateDimension()
         */
        virtual void control(Eigen::VectorXf& output, const Eigen::VectorXf& current_state) = 0;

        /**
         * Resets the controller to its initial state.
         */
        virtual void reset() = 0;

        /**
         * Returns the currently set state dimension.
         * @return state dimension
         */
        virtual unsigned int getStateDimension() = 0;

        /**
         * Sets the state dimension and resets the controller.
         * @param dim - state dimension
         */
        virtual void setStateDimension(unsigned int dim) = 0;
    };

    /**
     * A PID controller for a one-dimensional state.
     */
    class PIDController : public Controller {
    public:
        PIDController(float kp=1.0, float ki=0.1, float kd=0.0);
        virtual ~PIDController();
        void setKp(float kp);
        void setKi(float ki);
        void setKd(float kd);
        void setGains(float kp, float ki, float kd);
        virtual void setTarget(float target_state) override;
        virtual float getTarget() const override;
        virtual bool isTargetSatisfied(float current_state, float threshold=0.001f) const override;
        virtual float control(float current_state) override;
        virtual void reset() override;
    private:
        float _target;
        float _kp;
        float _ki;
        float _kd;
        float _integral_part;
        float _prev_error;
    };

    /**
     * A simple multi-dimensional controller where each degree of freedom is controlled
     * independently by one PID controller.
     */
    class IndependentMDPIDController : public MDController {
    public:
        IndependentMDPIDController(float kp, float ki, float kd);
        ~IndependentMDPIDController();

        virtual void setTarget(const Eigen::VectorXf& target_state) override;
        virtual void getTarget(Eigen::VectorXf& target_state) const override;
        virtual void control(Eigen::VectorXf& output, const Eigen::VectorXf& current_state) override;
        virtual void reset() override;
        virtual bool isTargetSatisfied(Eigen::VectorXf& current_state, float threshold=0.001f) const override;
        virtual void setGains(const Eigen::VectorXf& kps, const Eigen::VectorXf& kis, const Eigen::VectorXf& kds);
        virtual void setGains(float kp, float ki, float kd);
        virtual void setKps(const Eigen::VectorXf& kps);
        virtual void setKp(float kp);
        virtual void setKis(const Eigen::VectorXf& kis);
        virtual void setKi(float ki);
        virtual void setKds(const Eigen::VectorXf& kds);
        virtual void setKd(float kd);
        virtual unsigned int getStateDimension() override;
        virtual void setStateDimension(unsigned int dim) override;

    private:
        std::vector<PIDController> _controllers;
        float _default_kp;
        float _default_ki;
        float _default_kd;
    };

    /**
     * Interface for robot velocity controller.
     */
    class RobotVelocityController {
    public:
        virtual ~RobotVelocityController() = 0;
        virtual unsigned int getTargetDimension() = 0;
        virtual void setTargetVelocity(const Eigen::VectorXf& velocity) = 0;
        virtual RobotPtr getRobot() const = 0;
        /**
         * Clone this robot controller given that the controller will operate on the
         * robot with the same name in the given world. This allows you to clone this controller
         * and run it on a cloned world.
         * @param world
         * @return
         */
        virtual RobotVelocityControllerPtr clone(WorldPtr world) = 0;
        virtual bool control(const Eigen::VectorXf& positions,
                             const Eigen::VectorXf& velocities,
                             float timestep,
                             RobotConstPtr robot,
                             Eigen::VectorXf& output) = 0;
    };

    /**
     * Robot position controller - uses an Independent MDPIDController to control the position of a robot.
     */
    class RobotPositionController {
    public:
        RobotPositionController(RobotPtr robot, RobotVelocityControllerPtr velocity_controller);
        ~RobotPositionController();
        void setTargetPosition(const Eigen::VectorXf& position);
        virtual RobotPtr getRobot() const;
        bool control(const Eigen::VectorXf& positions,
                     const Eigen::VectorXf& velocities,
                     float timestep,
                     RobotConstPtr robot,
                     Eigen::VectorXf& output);
        IndependentMDPIDController& getPIDController();
    private:
        IndependentMDPIDController _pid_controller;
        RobotVelocityControllerPtr _velocity_controller;
        RobotWeakPtr _robot;
    };

}

#endif //SIM_ENV_CONTROLLER_H

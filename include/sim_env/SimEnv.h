//
// Created by joshua on 4/19/17.
//

#ifndef SIM_ENV_H
#define SIM_ENV_H

// STL imports
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <mutex>
#include <functional>

#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * This header file contains the definition of SimEnv. The idea behind SimEnv is to provide
 * a common interface for different robot simulators. This allows the development of planning
 * algorithms that do not have any strong dependencies on the underlying simulator implementation.
 * WARNING: Despite this, it is assumed that there is always only one SimEnv implementation used
 * in a process at a time. In other words, you should not mix multiple simulator implementations.
 * This means that implementations may, for instance, only provide collision checks with objects
 * from the same simulator implementation. Technically, this breaks the inheritance contract.
 */
namespace sim_env {

    class Logger;
    typedef std::shared_ptr<Logger> LoggerPtr;
    typedef std::weak_ptr<Logger> LoggerWeakPtr;
    typedef std::shared_ptr<const Logger> LoggerConstPtr;
    typedef std::weak_ptr<const Logger> LoggerConstWeakPtr;

    class Logger {
    public:
        enum class LogLevel {
            Debug=0, Info=1, Warn=2, Error=3
        };
        virtual ~Logger() = 0;
        virtual void setLevel(LogLevel lvl) = 0;
        virtual LogLevel getLevel() const = 0;
        virtual void logErr(const std::string& msg, const std::string& prefix="") const = 0;
        virtual void logInfo(const std::string& msg, const std::string& prefix="") const = 0;
        virtual void logWarn(const std::string& msg, const std::string& prefix="") const = 0;
        virtual void logDebug(const std::string& msg, const std::string& prefix="") const = 0;
        virtual void log(const std::string& msg, LogLevel level, const std::string& prefix="") const = 0;
    };

    class DefaultLogger : public Logger {
    public:
        DefaultLogger(const DefaultLogger& logger) = delete;
        void operator=(const DefaultLogger& logger) = delete;

        static LoggerPtr getInstance() {
            // this is thread safe in C++11
            static LoggerPtr pointer(new DefaultLogger());
            return pointer;
        }

        void setLevel(LogLevel lvl) override {
            _lvl = lvl;
        }

        LogLevel getLevel() const override {
            return _lvl;
        }

        void logErr(const std::string &msg, const std::string &prefix="") const override {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_lvl <= LogLevel::Error) {
                std::cout << "\033[1;31m[Error] " << prefix << " " << msg << " \033[0m " << std::endl;
            }
        }

        void logInfo(const std::string &msg, const std::string &prefix="") const override {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_lvl <= LogLevel::Info) {
                std::cout << "\033[1;32m[Info] " << prefix << " " << msg << " \033[0m " << std::endl;
            }
        }

        void logWarn(const std::string &msg, const std::string &prefix="") const override {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_lvl <= LogLevel::Warn) {
                std::cout << "\033[1;33m[Warning] " << prefix << " " << msg << " \033[0m " << std::endl;
            }
        }

        void logDebug(const std::string &msg, const std::string &prefix="") const override {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_lvl <= LogLevel::Debug) {
                std::cout << "\033[1;35m[Debug] " << prefix << " " << msg << " \033[0m " << std::endl;
            }
        }

        void log(const std::string& msg, LogLevel level, const std::string &prefix="") const override {
            switch (level) {
                case LogLevel::Info:
                    logInfo(msg, prefix);
                    break;
                case LogLevel::Error:
                    logErr(msg, prefix);
                    break;
                case LogLevel::Debug:
                    logDebug(msg, prefix);
                    break;
                case LogLevel::Warn:
                    logWarn(msg, prefix);
                    break;
            }
        }

    private:
        LogLevel _lvl;
        DefaultLogger() : _lvl(LogLevel::Info) {}
        mutable std::mutex _mutex;
    };

    enum class EntityType {
        Object, Robot, Joint, Link
    };

    class World;
    typedef std::shared_ptr<World> WorldPtr;
    typedef std::weak_ptr<World> WorldWeakPtr;
    typedef std::shared_ptr<const World> WorldConstPtr;
    typedef std::weak_ptr<const World> WorldConstWeakPtr;

    class Entity;
    typedef std::shared_ptr<Entity> EntityPtr;
    typedef std::weak_ptr<Entity> EntityWeakPtr;
    typedef std::shared_ptr<const Entity> EntityConstPtr;
    typedef std::weak_ptr<const Entity> EntityConstWeakPtr;

    class Object;
    typedef std::shared_ptr<Object> ObjectPtr;
    typedef std::weak_ptr<Object> ObjectWeakPtr;
    typedef std::shared_ptr<const Object> ObjectConstPtr;
    typedef std::weak_ptr<const Object> ObjectConstWeakPtr;

    class Robot;
    typedef std::shared_ptr<Robot> RobotPtr;
    typedef std::weak_ptr<Robot> RobotWeakPtr;
    typedef std::shared_ptr<const Robot> RobotConstPtr;
    typedef std::weak_ptr<const Robot> RobotConstWeakPtr;

    class Link;
    typedef std::shared_ptr<Link> LinkPtr;
    typedef std::weak_ptr<Link> LinkWeakPtr;
    typedef std::shared_ptr<const Link> LinkConstPtr;
    typedef std::weak_ptr<const Link> LinkConstWeakPtr;

    class Joint;
    typedef std::shared_ptr<Joint> JointPtr;
    typedef std::weak_ptr<Joint> JointWeakPtr;
    typedef std::shared_ptr<const Joint> JointConstPtr;
    typedef std::weak_ptr<const Joint> JointConstWeakPtr;

    struct Contact {
        ObjectWeakPtr object_a;
        ObjectWeakPtr object_b;
        LinkWeakPtr link_a;
        LinkWeakPtr link_b;
        Eigen::Vector3f contact_point; // in world frame
        Eigen::Vector3f contact_normal; // in world frame
    };

    /**
     * Base class for any entity stored in a SimEnv World.
     */
    class Entity {
    public:
        virtual ~Entity() = 0;
        /**
         * Returns the unique name of this entity.
         * @return string representing this entity's name.
         */
        virtual std::string getName() const = 0;


        /**
         * Returns the type of this entity.
         * @return the type encoded as enum SimEnvEntityType
         */
        virtual EntityType getType() const = 0;

        /**
         * Returns the transform of this object in world frame.
         * @return Eigen::Transform representing the pose of this object in world frame.
         */
        virtual Eigen::Affine3f getTransform() const = 0;

        /**
         * Returns the world object that this object belongs to.
         * @return the world
         */
        virtual WorldPtr getWorld() const = 0;

        /**
         * Returns the (constant) world object that this object belongs to.
         * @return the world
         */
        virtual WorldConstPtr getConstWorld() const = 0;

    protected:
        /**
         * Sets a new name for this entity.
         * @param name the new name
         */
        virtual void setName(const std::string& name) = 0;
    };

    struct DOFInformation {
        // TODO maybe include DOF type information: rotational vs translational
        unsigned int dof_index;
        Eigen::Array2f position_limits; // [min, max]
        Eigen::Array2f velocity_limits; // [min, max]
        Eigen::Array2f acceleration_limits; // [min, max]
    };

    class Link : public virtual Entity  {
    public:
        virtual ~Link() = 0;
        virtual ObjectPtr getObject() const = 0;
        virtual ObjectConstPtr getConstObject() const = 0;
        virtual void getChildJoints(std::vector<JointPtr>& child_joints) = 0;
        virtual void getConstChildJoints(std::vector<JointConstPtr>& child_joints) const = 0;
        virtual void getParentJoints(std::vector<JointPtr>& parent_joints) = 0;
        virtual void getConstParentJoints(std::vector<JointConstPtr>& parent_joints) const = 0;
        /**
         * Checks whether this Link collides with anything.
         * @return True if this Link collides with something
         */
        virtual bool checkCollision() = 0;
        /**
         * Checks whether this Link collides with anything.
         * @param list of contacts - all found contacts are stored in this list
         * @return True iff this Link collides with anything
         */
        virtual bool checkCollision(std::vector<Contact>& contacts) = 0;
        /**
         * Checks whether this Link collides with any of the other Links.
         * @param other_links - list of links to check collision with.
         * @return True iff this Link collides with any of the given Links.
         */
        virtual bool checkCollision(const std::vector<LinkPtr>& other_links) = 0;
        /**
         * Checks whether this Link collides with any of the other links.
         * @param other_links  - list of links to check collision with.
         * @param contacts - all detected contacts are saved in this list
         * @return True iff this Link collides with any of the given Links.
         */
        virtual bool checkCollision(const std::vector<LinkPtr>& other_links,
                                    std::vector<Contact>& contacts) = 0;
        /**
         * Checks whether this Link collides with any of the given objects.
         * @param other_objects - list of other objects.
         * @return True iff this Link collides with any of the given objects.
         */
        virtual bool checkCollision(const std::vector<ObjectPtr>& other_objects) = 0;
        /**
         * Checks whether this Link collides with any of the given objects.
         * @param other_objects - list of other objects.
         * @param contacts - all detected contacts are saved in this list
         * @return True iff this Link collides with any of the given objects.
         */
        virtual bool checkCollision(const std::vector<ObjectPtr>& other_objects,
                                    std::vector<Contact>& contacts) = 0;
    };

    class Joint : public virtual Entity {
    public:
        enum JointType {
            Revolute, Prismatic
        };
        virtual ~Joint() = 0;
        virtual float getPosition() const = 0;
        virtual void setPosition(float v) = 0;
        virtual float getVelocity() const = 0;
        virtual void setVelocity(float v) = 0;

        /**
         * Returns the joint index of this joint.
         * @return joint index
         */
        virtual unsigned int getJointIndex() const = 0;

        /**
         * Returns the degree-of-freedom index of this joint.
         * This may be different from the joint_index by some constant offset.
         * @see Object::getJointDOF(..)
         * @return joint index
         */
        virtual unsigned int getDOFIndex() const = 0;
        virtual JointType getJointType() const = 0;
        virtual LinkPtr getChildLink()  const = 0;
        virtual LinkPtr getParentLink() const = 0;
        virtual ObjectPtr getObject() const = 0;
        virtual ObjectConstPtr getConstObject() const = 0;
        virtual Eigen::Array2f getPositionLimits() const = 0;
        virtual void getPositionLimits(Eigen::Array2f& limits) const = 0;
        virtual Eigen::Array2f getVelocityLimits() const = 0;
        virtual void getVelocityLimits(Eigen::Array2f& limits) const = 0;
        virtual Eigen::Array2f getAccelerationLimits() const = 0;
        virtual void getAccelerationLimits(Eigen::Array2f& limits) const = 0;
        virtual DOFInformation getDOFInformation() const = 0;
        virtual void getDOFInformation(DOFInformation& info) const = 0;
    };

    struct ObjectState {
        Eigen::VectorXf dof_positions; // positions of ALL DOFs
        Eigen::VectorXf dof_velocities; // velocities of ALL DOFs
        Eigen::Affine3f pose; // pose/transform of the object
        Eigen::VectorXi active_dofs; // currently active DOFs
    };

    class Object : public virtual Entity {
    public:
        virtual ~Object() = 0;
        /**
         * Sets the transform of this object in world frame.
         * @param tf Eigen::Transform representing the new pose of this object
         */
        virtual void setTransform(const Eigen::Affine3f& tf) = 0;

        /**
         * Set the active degrees of freedom for this object.
         * @param indices list of active DOF indices
         */
        virtual void setActiveDOFs(const Eigen::VectorXi& indices) = 0;

        /**
         * Returns a list of the currently active degrees of freedom for this object.
         */
        virtual Eigen::VectorXi getActiveDOFs() const = 0;

        /**
         * Returns the number of active degrees of freedom.
         */
        virtual unsigned int getNumActiveDOFs() const = 0;

        /**
         * Returns all degree of freedom indices this object has.
         * For instance, if this object is a non-static rigid body, this function returns [0,1,2,3,4,5],
         * where 0 - x, 1 - y, 2 - z, 3 - rx, 4 - ry, 5 - rz.
         * @return a list containing all indices for all degrees of freedom.
         */
        virtual Eigen::VectorXi getDOFIndices() const = 0;

        /**
         * Returns the total number of degrees of freedom this object has.
         * @return maximal number of degrees of freedom.
         */
        virtual unsigned int getNumDOFs() const = 0;

        /**
         * Returns the number of degrees of freedom for the pose of this object.
         * @return 0 if this object is static, otherwise the number of DOFs for the pose of the object.
         */
        virtual unsigned int getNumBaseDOFs() const = 0;

        /**
         * Returns information on the specified degree of freedom.
         * @param dof_index - index of the degree of freedom
         * @return info
         */
        virtual DOFInformation getDOFInformation(unsigned int dof_index) const = 0;

        /**
         * Retrieves information on the specified degree of freedom.
         * @param dof_index - index of the degree of freedom
         * @param info - variable to store output in
         */
        virtual void getDOFInformation(unsigned int dof_index, DOFInformation& info) const = 0;

        /**
         * Get the current DoF position values of this object.
         * In case of a rigid object, this would be x,y,z,rx,ry,rz.
         * In case of a robot, this would additionally include all its joint positions.
         * @param indices a vector containing which DoFs to return. It returns the active DoFs, if the vector is empty.
         * @return vector containing the requested DoF positions.
         */
        virtual Eigen::VectorXf getDOFPositions(const Eigen::VectorXi& indices=Eigen::VectorXi()) const = 0;

        /**
         * Get the dof position limits of this object.
         * If a DoF is unlimited, the corresponding limits are (std::numeric_limits<float>::min(), std::numeric_limits<float>::max())
         * @param indices a vector containing which limits to return. It returns the limits of the active DoFs, if the vector is empty.
         * @return array containing the limits, where each row is a pair (min, max)
         */
        virtual Eigen::ArrayX2f getDOFPositionLimits(const Eigen::VectorXi& indices=Eigen::VectorXi()) const = 0;

        /**
         * Retrieves this object's current state. See ObjectState for the definition of an object's state.
         * @param object_state
         */
        virtual void getState(ObjectState& object_state) const = 0;
        virtual ObjectState getState() const = 0;

        /**
         * Sets the state of this object.
         * @param object_state
         */
        virtual void setState(const ObjectState& object_state) = 0;

        /**
         * Set the current DoF position values of this object. Also see getDOFPositions.
         * @param indices a vector containing which DoFs to return. It sets the active DoFs, if no indices are given.
         */
        virtual void setDOFPositions(const Eigen::VectorXf& values, const Eigen::VectorXi& indices=Eigen::VectorXi()) = 0;

        /**
         * Get the current DoF velocity values of this object.
         * @param indices a vector containing which DoF velocities to return. It returns the active DoFs, if the vector is empty.
         * @return vector containing the requested DoF velocities.
         */
        virtual Eigen::VectorXf getDOFVelocities(const Eigen::VectorXi& indices=Eigen::VectorXi()) const = 0;

        /**
         * Get the dof velocity limits of this object.
         * If a DoF is unlimited, the corresponding limits are (std::numeric_limits<float>::min(), std::numeric_limits<float>::max())
         * @param indices a vector containing which limits to return. It returns the limits of the active DoFs, if the vector is empty.
         * @return array containing the limits, where each row is a pair (min, max)
         */
        virtual Eigen::ArrayX2f getDOFVelocityLimits(const Eigen::VectorXi& indices=Eigen::VectorXi()) const = 0;

        /**
         * Get the dof acceleration limits of this object.
         * If a DoF is unlimited, the corresponding limits are (std::numeric_limits<float>::min(), std::numeric_limits<float>::max())
         * @param indices a vector containing which limits to return. It returns the limits of the active DoFs, if the vector is empty.
         * @return array containing the limits, where each row is a pair (min, max)
         */
        virtual Eigen::ArrayX2f getDOFAccelerationLimits(const Eigen::VectorXi& indices=Eigen::VectorXi()) const = 0;

        /**
         * Set the current DoF velocity values of this object. Also see getDOFPositions.
         * @param indices a vector containing which DoF velocities to return. It sets the active DoFs, if the vector is empty.
         */
        virtual void setDOFVelocities(const Eigen::VectorXf& values, const Eigen::VectorXi& indices=Eigen::VectorXi()) = 0;

        /**
         * Returns whether this object is static, i.e. not movable by any finite force.
         * @return bool whether this object is static.
         */
        virtual bool isStatic() const = 0;

        /**
         * Retrieves all links of this object and adds them to the given list.
         * @warning This method returns shared_ptrs. Do not store these references beyond the lifespan of this object.
         * @param links a vector in which all links are stored (not reset).
         */
        virtual void getLinks(std::vector<LinkPtr>& links) = 0;
        virtual void getLinks(std::vector<LinkConstPtr>& links) const = 0;
        virtual LinkPtr getLink(const std::string& link_name) = 0;
        virtual LinkConstPtr getConstLink(const std::string& link_name) const = 0;
        /**
         * Returns the base link of the object. The base link is the root of any kinematic chain
         * of the object.
         * @return base link
         */
        virtual LinkPtr getBaseLink() = 0;

        /**
         * Retrieves all joints of this object and adds them to the given list.
         * @warning This method returns shared_ptrs. Do not store these references beyond the lifespan of this object.
         * @param joints a vector in which all joints are stored (not reset).
         */
        virtual void getJoints(std::vector<JointPtr>& joints) = 0;
        virtual void getJoints(std::vector<JointConstPtr>& joints) const = 0;

        /**
         * Retrieves the joint with the provided name.
         * @param joint_name name of the joint to retrieve.
         * @return valid pointer to joint with name joint_name, if it exists, else nullptr
         */
        virtual JointPtr getJoint(const std::string& joint_name) = 0;

        /**
         * Retrieves the joint with the provided joint index.
         * @param joint_index index of the joint to retrieve
         * @return valid pointer to joint with index joint_idx, if it exists, else nullptr
         */
        virtual JointPtr getJoint(unsigned int joint_idx) = 0;

        /**
         * Retrieves the joint with the provided degree-of-freedom index.
         * The degree-of-freedom index differs from the joint index if this object is
         * not static. In this case the first 1 <= k <= 6 degrees of freedom describe
         * the pose of the object, and all other indices > k describe joints.
         * In other words, the relationship of degree-of-freedom index and joint index is:
         * joint index = degree-of-freedom index + k
         * where k is the number of free pose parameters (e.g x, y, orientation).
         * @param joint_index index of the joint to retrieve
         * @return valid pointer to joint with index joint_idx, if it exists, else nullptr
         */
        virtual JointPtr getJointFromDOFIndex(unsigned int dof_idx) = 0;
        virtual JointConstPtr getConstJoint(const std::string& joint_name) const = 0;
        virtual JointConstPtr getConstJoint(unsigned int joint_idx) const = 0;
        virtual JointConstPtr getConstJointFromDOFIndex(unsigned int dof_idx) const = 0;

        /**
         * Checks whether this Object collides with anything.
         * @return True if this Object collides with something
         */
        virtual bool checkCollision() = 0;

        /**
         * Checks whether this Object collides with anything.
         * @param list of contacts - all found contacts are stored in this list
         * @return True iff this Object collides with anything
         */
        virtual bool checkCollision(std::vector<Contact>& contacts) = 0;

        /**
         * Checks whether this Object collides with the provided object.
         * @param object - the other object
         * @return True iff this Object collides with the other object
         */
        virtual bool checkCollision(ObjectPtr other_object) = 0;
        virtual bool checkCollision(ObjectPtr other_object, std::vector<Contact>& contacts) = 0;

        /**
         * Checks whether this Object collides with any of the given objects.
         * @param other_objects - list of other objects.
         * @return True iff this Object collides with any of the given objects.
         */
        virtual bool checkCollision(const std::vector<ObjectPtr>& other_objects) = 0;
        /**
         * Checks whether this Obejct collides with any of the given objects.
         * @param other_objects - list of other objects.
         * @param contacts - all detected contacts are saved in this list
         * @return True iff this Object collides with any of the given objects.
         */
        virtual bool checkCollision(const std::vector<ObjectPtr>& other_objects,
                                    std::vector<Contact>& contacts) = 0;
    };

    /**
     * A robot is essentially an actuated object.
     */
    class Robot : public virtual Object {
    public:
        /** The callback should have signature:
        *   bool callback(const Eigen::VectorXf& positions, const Eigen::VectorXf& velocities, float timestep,
        *                 RobotPtr robot, Eigen::VectorXf& output);
        */
        typedef std::function<bool(const Eigen::VectorXf&, const Eigen::VectorXf&, float,
                                   RobotConstPtr, Eigen::VectorXf&)> ControlCallback;
        /**
         * Register a controller callback. See typedef of ControlCallback for details about the signature.
         * The provided callback is called in every simulation step to apply controls (forces and torques)
         * to the robot's degrees of freedom.
         * @param controll_fn callback function with signature ControlCallback that
         */
        virtual void setController(ControlCallback controll_fn) = 0;
        virtual ~Robot() = 0;
    };


    class WorldViewer {
    public:

        //TODO define all drawing functions here; provide support for setting colors and width
        virtual void drawFrame(const Eigen::Affine3f& transform, float length=1.0f, float width=0.1f) = 0;

    };

    /* Typedefs for shared pointers on this class */
    typedef std::shared_ptr<WorldViewer> WorldViewerPtr;
    typedef std::shared_ptr<const WorldViewer> WorldViewerConstPtr;

    typedef std::map<std::string, ObjectState> WorldState;

    class World { // public std::enable_shared_from_this<World>
    public:
        virtual ~World() = 0;
        /**
         * Loads the world from the given file.
         * Note, that the supported file formats depend on the underlying implementation.
         * @param path A string containing a path to a world file to load.
         */
        virtual void loadWorld(const std::string& path) = 0;

        /**
         * Returns the robot with given name, if available.
         * @warning This method returns a shared_ptr. Do not store this reference beyond the lifespan of this world.
         * @param name The unique name of the robot to return
         * @return Pointer to the robot with given name or nullptr if not available
         */
        virtual RobotPtr getRobot(const std::string& name) = 0;
        virtual RobotConstPtr getRobotConst(const std::string& name) const = 0;

        /**
         * Returns the object with given name, if available. Use getRobot or set exclude_robot to false to retrieve a robot.
         * @warning This method returns a shared_ptr. Do not store this reference beyond the lifespan of this world.
         * @param name The unique name of the object to return
         * @return Pointer to the object with given name or nullptr if not available
         */
        virtual ObjectPtr getObject(const std::string& name, bool exclude_robot=true) = 0;
        virtual ObjectConstPtr getObjectConst(const std::string& name, bool exclude_robot=true) const = 0;

        /**
         * Returns all objects stored in the world.
         * @warning This method returns shared_ptrs. Do not store these references beyond the lifespan of this world.
         * @param objects List to fill with objects. The list is not cleared.
         * @param exclude_robots Flag whether to include or exclude robots.
         */
        virtual void getObjects(std::vector<ObjectPtr>& objects, bool exclude_robots=true) = 0;
        virtual void getObjects(std::vector<ObjectConstPtr>& objects, bool exclude_robots=true) const = 0;

        /**
         * Returns all robots stored in the world.
         * @warning This method returns shared_ptrs. Do not store these references beyond the lifespan of this world.
         * @param objects List to fill with objects. The list is not cleared.
         */
        virtual void getRobots(std::vector<RobotPtr>& robots) = 0;
        virtual void getRobots(std::vector<RobotConstPtr>& robots) const = 0;

        /**
         * If the underlying world representation supports physics simulation,
         * this function issues a physics simulation step.
         * @param steps number of physics time steps to simulate.
         */
        virtual void stepPhysics(int steps=1) = 0;

        /**
         * Returns whether the underlying world representation supports physics simulation or not.
         * @return whether physics is supported or not.
         */
        virtual bool supportsPhysics() const = 0;

        /**
         * Sets the delta t (time) for the physics simulation.
         * @param physics_step Time in seconds to simulate per simulation step.
         */
        virtual void setPhysicsTimeStep(float physics_step) = 0;

        /**
         * Checks whether both provided objects collide.
         * @param object_a - first object
         * @param object_b - second object
         * @param contacts - (optional) all contacts are stored in this list
         * @return true iff there is a collision between both objects
         */
        virtual bool checkCollision(ObjectPtr object_a, ObjectPtr object_b) = 0;
        virtual bool checkCollision(ObjectPtr object_a, ObjectPtr object_b, std::vector<Contact>& contacts) = 0;

        /**
         * Checks whether the given object collides with the given link.
         * @param object_a - object
         * @param link_b - link
         * @param contacts - (optional) all contacts are stored in this list
         * @return true iff there is a collision between the object and link_b
         */
        virtual bool checkCollision(ObjectPtr object_a, LinkPtr link_b) = 0;
        virtual bool checkCollision(ObjectPtr object_a, LinkPtr link_b, std::vector<Contact>& contacts) = 0;

        /**
         * Checks whether the given object is in collision.
         * @param object
         * @param contacts - (optional) all contacts are stored in this list
         * @return true iff object is in collision with any other object
         */
        virtual bool checkCollision(ObjectPtr object) = 0;
        virtual bool checkCollision(ObjectPtr object, std::vector<Contact>& contacts) = 0;

        /**
         * Checks whether the given link is in collision.
         * @param link
         * @param contacts - (optional) all contacts are stored in this list
         * @return true iff link is in collision with any other object
         */
        virtual bool checkCollision(LinkPtr link) = 0;
        virtual bool checkCollision(LinkPtr link, std::vector<Contact>& contacts) = 0;

        /**
         * Checks whether the given link is in collision.
         * @param link
         * @param contacts - (optional) all contacts are stored in this list
         * @return true iff link is in collision with any other object
         */
        virtual bool checkCollision(LinkPtr link, const std::vector<LinkPtr>& other_links) = 0;
        virtual bool checkCollision(LinkPtr link, const std::vector<LinkPtr>& other_links,
                                    std::vector<Contact>& contacts) = 0;

        /**
         * Checks whether object_a is in collision with any of the provided objects.
         * @param object_a
         * @param other_objects
         * @param contacts - (optional) all contacts are stored in this list
         * @return true iff object_a collides with any of the provided objects in other_objects
         */
        virtual bool checkCollision(ObjectPtr object_a, const std::vector<ObjectPtr>& other_objects) = 0;
        virtual bool checkCollision(ObjectPtr object_a, const std::vector<ObjectPtr>& other_objects,
                                    std::vector<Contact>& contacts) = 0;

        /**
         * Checks whether link_a is in collision with any of the provided objects.
         * @param link_a
         * @param other_objects
         * @param contacts - (optional) all contacts are stored in this list
         * @return true iff link_a collides with any of the provided objects in other_objects
         */
        virtual bool checkCollision(LinkPtr link_a, const std::vector<ObjectPtr>& other_objects) = 0;
        virtual bool checkCollision(LinkPtr link_a, const std::vector<ObjectPtr>& other_objects,
                                    std::vector<Contact>& contacts) = 0;
        /**
         * Returns the currently set physics time step.
         */
        virtual float getPhysicsTimeStep() const = 0;

        /**
         * Returns an instance of graphical world viewer.
         * This function may also internally create the viewer first.
         * @return shared pointer to a world viewer showing this world.
         */
        virtual WorldViewerPtr getViewer() = 0;

        /**
         * Returns an instance of the logger used in the context of this environment.
         * @return shared pointer to a logger.
         */
        virtual LoggerPtr getLogger() = 0;
        virtual LoggerConstPtr getConstLogger() const = 0;

        /**
         * Retrieve the state of this world, i.e. all ObjectStates for all objects/robots
         */
        virtual WorldState getWorldState() const = 0;
        virtual void getWorldState(WorldState& state) const = 0;

        /**
         * Set the state of this world, i.e. the ObjectStates for the objects/robots in state
         * @return true iff setting state successful.
         */
        virtual bool setWorldState(WorldState& state) = 0;

        /**
         * Saves the current state on an internal state stack.
         * Note that this state is automatically dropped if a new world is loaded or
         * an object is added or removed.
         */
        virtual void saveState() = 0;

        /**
         * Restores the last saved state.
         * @return true iff there was a state to restore to
         */
        virtual bool restoreState() = 0;

        /**
         * Returns a mutex to lock this world.
         * Any function that changes the state of this world should lock it first.
         * @return recursive mutex for this world
         */
        virtual std::recursive_mutex& getMutex() const = 0;
    };
}

#endif //SIM_ENV_H

//
// Created by joshua on 4/19/17.
//

#ifndef SIM_ENV_H
#define SIM_ENV_H

// STL imports
#include <atomic>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
// Eigen imports
#include <Eigen/Dense>
#include <Eigen/Geometry>
// Boost imports
#include <boost/format.hpp>
// sim_env imports
#include <sim_env/Grid.h>

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
        Debug = 0,
        Info = 1,
        Warn = 2,
        Error = 3
    };
    virtual ~Logger() = 0;
    virtual void setLevel(LogLevel lvl) = 0;
    virtual LogLevel getLevel() const = 0;
    //        virtual std::ostream& getStream(LogLevel level, const std::string& prefix="") const = 0;
    virtual void logErr(const std::string& msg, const std::string& prefix = "") const = 0;
    virtual void logErr(const boost::format& bf, const std::string& prefix = "") const;
    virtual void logInfo(const std::string& msg, const std::string& prefix = "") const = 0;
    virtual void logInfo(const boost::format& bf, const std::string& prefix = "") const;
    virtual void logWarn(const std::string& msg, const std::string& prefix = "") const = 0;
    virtual void logWarn(const boost::format& bf, const std::string& prefix = "") const;
    virtual void logDebug(const std::string& msg, const std::string& prefix = "") const = 0;
    virtual void logDebug(const boost::format& bf, const std::string& prefix = "") const;
    virtual void log(const std::string& msg, LogLevel level, const std::string& prefix = "") const = 0;
    virtual void log(const boost::format& bf, LogLevel level, const std::string& prefix = "") const;
};

class DefaultLogger : public Logger {
public:
    DefaultLogger(const DefaultLogger& logger) = delete;
    void operator=(const DefaultLogger& logger) = delete;

    static LoggerPtr getInstance()
    {
        // this is thread safe in C++11
        static LoggerPtr pointer(new DefaultLogger());
        return pointer;
    }

    void setLevel(LogLevel lvl) override
    {
        _lvl = lvl;
    }

    LogLevel getLevel() const override
    {
        return _lvl;
    }

    void logErr(const std::string& msg, const std::string& prefix = "") const override
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_lvl <= LogLevel::Error) {
            std::cout << "\033[1;31m[Error] " << prefix << " " << msg << " \033[0m " << std::endl;
        }
    }

    void logInfo(const std::string& msg, const std::string& prefix = "") const override
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_lvl <= LogLevel::Info) {
            std::cout << "\033[1;32m[Info] " << prefix << " " << msg << " \033[0m " << std::endl;
        }
    }

    void logWarn(const std::string& msg, const std::string& prefix = "") const override
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_lvl <= LogLevel::Warn) {
            std::cout << "\033[1;33m[Warning] " << prefix << " " << msg << " \033[0m " << std::endl;
        }
    }

    void logDebug(const std::string& msg, const std::string& prefix = "") const override
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_lvl <= LogLevel::Debug) {
            std::cout << "\033[1;35m[Debug] " << prefix << " " << msg << " \033[0m " << std::endl;
        }
    }

    void log(const std::string& msg, LogLevel level, const std::string& prefix = "") const override
    {
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
    DefaultLogger()
        : _lvl(LogLevel::Info)
    {
    }
    mutable std::mutex _mutex;
};

enum class EntityType {
    Object,
    Robot,
    Joint,
    Link
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

struct BoundingBox {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3f min_corner;
    Eigen::Vector3f max_corner;

    BoundingBox()
    {
        min_corner.setZero();
        max_corner.setZero();
    }

    float getWidth()
    {
        return max_corner[0] - min_corner[0];
    }
    float getHeight()
    {
        return max_corner[1] - min_corner[1];
    }
    float getDepth()
    {
        return max_corner[2] - min_corner[2];
    }

    /**
         *  Returns the extents of this bounding box, which are (width/2, height/2, depth/2)
         */
    Eigen::Vector3f extents() const
    {
        return 0.5f * (max_corner - min_corner);
    }

    /**
         *  Returns the center of this bounding box.
         */
    Eigen::Vector3f center() const
    {
        return 0.5f * (min_corner + max_corner);
    }

    void merge(const BoundingBox& other)
    {
        for (unsigned int i = 0; i < min_corner.size(); ++i) {
            min_corner[i] = std::min(min_corner[i], other.min_corner[i]);
            max_corner[i] = std::max(max_corner[i], other.max_corner[i]);
        }
    }
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

struct Ball {
    Eigen::Vector3f center;
    float radius;
    Ball();
    Ball(Eigen::Vector3f center, float radius);
    Ball(const Ball& other);
    ~Ball();
    Ball& operator=(const Ball& other);
};

class Link : public virtual Entity {
public:
    virtual ~Link() = 0;
    virtual ObjectPtr getObject() const = 0;
    virtual ObjectConstPtr getConstObject() const = 0;
    virtual void getChildJoints(std::vector<JointPtr>& child_joints) = 0;
    virtual void getConstChildJoints(std::vector<JointConstPtr>& child_joints) const = 0;
    virtual JointPtr getParentJoint() = 0;
    virtual JointConstPtr getConstParentJoint() const = 0;

    /**
         * Provides a set of balls that approximate the shape of this link.
         * The ball positions are in global coordinates.
         * @param balls - a vector to be filled with all balls, balls are appended to the vector
         */
    virtual void getBallApproximation(std::vector<Ball>& balls) const = 0;
    /**
         * Provides a set of balls that approximate  the shape of this link.
         * The ball positions are in link frame.
         * @param balls - a vector to be filled with all balls, balls are appended to the vector
         */
    virtual void getLocalBallApproximation(std::vector<Ball>& balls) const = 0;
    /**
         * Updates the global positions of the given approximation balls.
         * @param balls - a vector containing a sufficient number of balls
         * @param start - start iterator pointing to the first ball to update
         * @param end - end itartor indicating where to stop. between end and stop there must be
         *              sufficiently many balls to update (as many as getBallApproximation is providing)
         */
    virtual void updateBallApproximation(std::vector<Ball>& balls,
        std::vector<Ball>::iterator& start,
        std::vector<Ball>::iterator& end) const = 0;
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
        std::vector<Contact>& contacts)
        = 0;
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
        std::vector<Contact>& contacts)
        = 0;

    virtual void setMass(float mass) = 0;
    virtual float getMass() const = 0;
    virtual void getCenterOfMass(Eigen::Vector3f& com) const = 0;
    virtual void getLocalCenterOfMass(Eigen::Vector3f& com) const = 0;
    virtual float getGroundFriction() const = 0;
    virtual void setGroundFriction(float coeff) = 0;
    /**
     *  Enable or disable this link. If disabled, the link can not collide with anything.
     * @param b_enable - True -> enabled, False -> disabled
     */
    virtual void setEnabled(bool b_enable) = 0;
    // returns whether this link is enabled
    virtual bool isEnabled() const = 0;
};

class Joint : public virtual Entity {
public:
    enum JointType {
        Revolute,
        Prismatic
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
    virtual LinkPtr getChildLink() const = 0;
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
    virtual Eigen::VectorXf getDOFPositions(const Eigen::VectorXi& indices = Eigen::VectorXi()) const = 0;

    /**
         * Get the dof position limits of this object.
         * If a DoF is unlimited, the corresponding limits are (std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max())
         * @param indices a vector containing which limits to return. It returns the limits of the active DoFs, if the vector is empty.
         * @return array containing the limits, where each row is a pair (min, max)
         */
    virtual Eigen::ArrayX2f getDOFPositionLimits(const Eigen::VectorXi& indices = Eigen::VectorXi()) const = 0;

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
    virtual void setDOFPositions(const Eigen::VectorXf& values, const Eigen::VectorXi& indices = Eigen::VectorXi()) = 0;

    /**
         * Get the current DoF velocity values of this object.
         * @param indices a vector containing which DoF velocities to return. It returns the active DoFs, if the vector is empty.
         * @return vector containing the requested DoF velocities.
         */
    virtual Eigen::VectorXf getDOFVelocities(const Eigen::VectorXi& indices = Eigen::VectorXi()) const = 0;

    /**
         * Get the dof velocity limits of this object.
         * If a DoF is unlimited, the corresponding limits are (std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max())
         * @param indices a vector containing which limits to return. It returns the limits of the active DoFs, if the vector is empty.
         * @return array containing the limits, where each row is a pair (min, max)
         */
    virtual Eigen::ArrayX2f getDOFVelocityLimits(const Eigen::VectorXi& indices = Eigen::VectorXi()) const = 0;

    /**
         * Get the dof acceleration limits of this object.
         * If a DoF is unlimited, the corresponding limits are (std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max())
         * @param indices a vector containing which limits to return. It returns the limits of the active DoFs, if the vector is empty.
         * @return array containing the limits, where each row is a pair (min, max)
         */
    virtual Eigen::ArrayX2f getDOFAccelerationLimits(const Eigen::VectorXi& indices = Eigen::VectorXi()) const = 0;

    /**
         * Set the current DoF velocity values of this object. Also see getDOFPositions.
         * @param indices a vector containing which DoF velocities to return. It sets the active DoFs, if the vector is empty.
         */
    virtual void setDOFVelocities(const Eigen::VectorXf& values, const Eigen::VectorXi& indices = Eigen::VectorXi()) = 0;

    /**
         * Returns whether this object is static, i.e. not movable by any finite force.
         * @return bool whether this object is static.
         */
    virtual bool isStatic() const = 0;

    /**
         * Provides a set of balls that approximate the shape of this object.
         * The ball positions are in global coordinates.
         * @param balls - a vector to be filled with all balls
         *               Any ball in the given vector is reused, i.e. the position updated. If the number of balls
         *              does not suffice, additional balls are appended. If there are too many, the size is reduced.
         */
    virtual void getBallApproximation(std::vector<Ball>& balls) const = 0;
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
        std::vector<Contact>& contacts)
        = 0;
    /**
         * Checks whether this object is at rest, i.e. all dof velocities are below the given threshold.
         * @param threshold
         * @return true iff this object is at rest
         */
    virtual bool atRest(float threshold = 0.0001f) const = 0;

    virtual float getMass() const = 0;
    virtual float getInertia() const = 0; // TODO this should return a matrix
    virtual BoundingBox getLocalAABB() const = 0;
    virtual float getGroundFriction() const = 0;
    /**
     * Enable or disable this object. A disabled object does not physically interact (i.e. collide)
     * with any other object. By default, every object is enabled.
     * @param b_enable - True = disable, False = Enable
     */
    virtual void setEnabled(bool b_enable) = 0;
    // returns whether this object is enabled. An object is enabled if at least one of its links is enabled
    virtual bool isEnabled() const = 0;
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
        RobotConstPtr, Eigen::VectorXf&)>
        ControlCallback;
    /**
         * Register a controller callback. See typedef of ControlCallback for details about the signature.
         * The provided callback is called in every simulation step to apply controls (forces and torques)
         * to the robot's degrees of freedom.
         * @param controll_fn callback function with signature ControlCallback that
         */
    virtual void setController(ControlCallback controll_fn) = 0;
    // /**
    //  * Returns for each active degree of freedom a delta_i > 0 that can be used to numerically compute
    //  */
    // virtual void getGradientDeltas(Eigen::VectorXf& deltas) const = 0;
    virtual ~Robot() = 0;
};

class WorldViewer {
public:
    class Handle {
    public:
        Handle(bool valid_handle = true);
        Handle(const Handle& other);
        ~Handle();
        Handle& operator=(const Handle& other);
        unsigned int getID() const;
        bool isValid() const;

    private:
        static std::atomic_uint _global_id_counter;
        unsigned int _id;
    };

    class ImageRenderer {
        /**
         * An ImageRenderer allows you to render camera images of a scene. 
         * The underlying implementation guarantees that an ImageRenderer can also be used
         * in non-GUI threads.
         */
    public:
        virtual ~ImageRenderer() = 0;

        /**
         *  Render the scene from the current camera view to an image and store this image under the given name.
         *  The underlying implementation guarantees that this method is thread-safe.
         *  @param filename - path + name of where to store the image. The image format may depend on the implementation.
         *  @param width - width in pixels of the image
         *  @param height - height in pixels
         *  @param include_drawings - if true, also render additional drawings in image, else not
         */
        virtual bool renderImage(const std::string& filename,
                                 unsigned int width, unsigned int height, bool include_drawings = false) = 0;

        // TODO add renderImage from different camera transformations if needed
        /**
         *  Position the camera such that all bodies in the scene are visible.
         * @param include_drawings - if true, it also ensures that all user drawings are visible
         */
        virtual void centerCamera(bool include_drawings = false) = 0;
        /**
         *  Resets the camera to default view.
         */
        virtual void resetCamera() = 0;

        /**
         * Sets whether to show the object/robot with the given name when rendering images.
         * By default all objects/robots are shown.
         * @param name - name of the object to show or hide
         * @param visible - if true, show it, else hide. 
         */
        virtual void setVisible(const std::string& name, bool visible) = 0;

        /**
         * Set the color of the object/robot with the given name.
         * By default all objects/robots are shown.
         * @param name - name of the object to show or hide
         * @param visible - if true, show it, else hide. 
         */
        virtual void setColor(const std::string& name, const Eigen::Vector4f& color) = 0;
        /**
         * Draw a coordinate frame.
         * @param transform - the transformation matrix from the target frame to world frame
         * @param length - length of arrows
         * @param widt - width of arrows.
         */
        virtual Handle drawFrame(const Eigen::Affine3f& transform, float length = 1.0f, float width = 0.1f) = 0;
        /**
             * Draw a box at the provided position with the given extents.
             * The box spans from pos to pos + extents
             * @param pos - box position (with minimal coordinates)
             * @param extent - (width, depth, height)
             * @param color - rgba color (in range [0,1]^4)
             * @param solid - flag whether to draw a solid or non-solid box
             * @param edge_width - thickness of lines
             */
        virtual Handle drawBox(const Eigen::Vector3f& pos, const Eigen::Vector3f& extent,
            const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
            bool solid = false, float edge_width = 0.1f)
            = 0;

        /**
             * Draws a line from position start to position end.
             * @param start  - position where the line segment should start
             * @param end  - position where the line segment should end
             * @param color - rgba color (in range [0,1]^4)
             * @param width - width of the line
             * @return handle to delete the line again
             */
        virtual Handle drawLine(const Eigen::Vector3f& start, const Eigen::Vector3f& end,
            const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
            float width = 0.1f)
            = 0;

        /**
             * Draws a sphere with the given radius centered at center.
             * @param center - center position of the sphere.
             * @param radius - radius of the sphere.
             * @param color - (optional) rbda color of the sphere
             * @param width - (optional) width of the line
             * TODO: some option to only draw a 2d circle
             * @return handle to delete this sphere again
             */
        virtual Handle drawSphere(const Eigen::Vector3f& center, float radius,
            const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
            float width = 0.1f)
            = 0;

        /**
             * Draws the given voxel grid. The grid is assumed to store a color for each cell.
             * @param grid - a voxel grid that stores the color for each voxel
             * @param old_handle (optional) - a handle to a previously drawn instance of a voxel grid that is supposed to
             *      be replaced by this new drawing. Providing this may save resources, but you need to ensure that the new
             *      grid has the same dimensions as the previous one.
             * @return handle to delete this grid again
             */
        virtual Handle drawVoxelGrid(const grid::VoxelGrid<float, Eigen::Vector4f>& grid, const Handle& old_handle = Handle(false)) = 0;

        /**
         * Remove the drawing with the given handle.
         * If there is no drawing for that handle, this is a no-op.
         */
        virtual void removeDrawing(const Handle& handle) = 0;

        /**
         * Remove all drawings.
         */
        virtual void removeAllDrawings() = 0;
    };
    typedef std::shared_ptr<ImageRenderer> ImageRendererPtr;
    typedef std::shared_ptr<const ImageRenderer> ImageRendererConstPtr;
    typedef std::weak_ptr<ImageRenderer> ImageRendererWeakPtr;
    typedef std::weak_ptr<const ImageRenderer> ImageRendererWeakConstPtr;

    virtual ~WorldViewer()
        = 0;

    //TODO define all drawing functions here; provide support for setting colors and width
    /**
         * Draw a coordinate frame.
         * @param transform - the transformation matrix from the target frame to world frame
         * @param length - length of arrows
         * @param widt - width of arrows.
         */
    virtual Handle drawFrame(const Eigen::Affine3f& transform, float length = 1.0f, float width = 0.1f) = 0;
    /**
         * Draw a box at the provided position with the given extents.
         * The box spans from pos to pos + extents
         * @param pos - box position (with minimal coordinates)
         * @param extent - (width, depth, height)
         * @param color - rgba color (in range [0,1]^4)
         * @param solid - flag whether to draw a solid or non-solid box
         * @param edge_width - thickness of lines
         */
    virtual Handle drawBox(const Eigen::Vector3f& pos, const Eigen::Vector3f& extent,
        const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
        bool solid = false, float edge_width = 0.1f)
        = 0;

    /**
         * Draws a line from position start to position end.
         * @param start  - position where the line segment should start
         * @param end  - position where the line segment should end
         * @param color - rgba color (in range [0,1]^4)
         * @param width - width of the line
         * @return handle to delete the line again
         */
    virtual Handle drawLine(const Eigen::Vector3f& start, const Eigen::Vector3f& end,
        const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
        float width = 0.1f)
        = 0;

    /**
         * Draws a sphere with the given radius centered at center.
         * @param center - center position of the sphere.
         * @param radius - radius of the sphere.
         * @param color - (optional) rbda color of the sphere
         * @param width - (optional) width of the line
         * TODO: some option to only draw a 2d circle
         * @return handle to delete this sphere again
         */
    virtual Handle drawSphere(const Eigen::Vector3f& center, float radius,
        const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
        float width = 0.1f)
        = 0;

    /**
         * Draws the given voxel grid. The grid is assumed to store a color for each cell.
         * @param grid - a voxel grid that stores the color for each voxel
         * @param old_handle (optional) - a handle to a previously drawn instance of a voxel grid that is supposed to
         *      be replaced by this new drawing. Providing this may save resources, but you need to ensure that the new
         *      grid has the same dimensions as the previous one.
         * @return handle to delete this grid again
         */
    virtual Handle drawVoxelGrid(const grid::VoxelGrid<float, Eigen::Vector4f>& grid, const Handle& old_handle = Handle(false)) = 0;

    /**
     *  Render the scene from the current camera view to an image and store this image under the given name.
     *  NOTE: This function may not be thread-safe. For thread-safe image rendering use an ImageRenderer.
     *  @param filename - path + name of where to store the image. The image format may depend on the implementation.
     *  @param width - width in pixels of the image
     *  @param height - height in pixels
     *  @param include_drawings - if true, also render additional drawings in image, else not
     */
    virtual bool renderImage(const std::string& filename, unsigned int width, unsigned int height, bool include_drawings = false) = 0;

    /**
     *  Position the camera such that all bodies in the scene are visible.
     * @param include_drawings - if true, it also ensures that all user drawings are visible
     */
    virtual void centerCamera(bool include_drawings = false) = 0;
    /**
     *  Resets the camera to default view.
     */
    virtual void resetCamera() = 0;

    virtual WorldPtr getWorld() const = 0;
    virtual void removeDrawing(const Handle& handle) = 0;
    virtual void removeAllDrawings() = 0;

    /**
     * Set the color of the object/robot with the given name.
     * By default all objects/robots are shown.
     * @param name - name of the object to show or hide
     * @param visible - if true, show it, else hide. 
     */
    virtual void setColor(const std::string& name, const Eigen::Vector4f& color) = 0;
    /**
     * Sets whether to show the object/robot with the given name in the viewer.
     * By default all objects/robots are shown.
     * @param name - name of the object to show or hide
     * @param visible - if true, show it, else hide. 
     */
    virtual void setVisible(const std::string& name, bool visible) = 0;
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
    virtual ObjectPtr getObject(const std::string& name, bool exclude_robot = true) = 0;
    virtual ObjectConstPtr getObjectConst(const std::string& name, bool exclude_robot = true) const = 0;

    /**
         * Returns all objects stored in the world.
         * @warning This method returns shared_ptrs. Do not store these references beyond the lifespan of this world.
         * @param objects List to fill with objects. The list is not cleared.
         * @param exclude_robots Flag whether to include or exclude robots.
         */
    virtual void getObjects(std::vector<ObjectPtr>& objects, bool exclude_robots = true) = 0;
    virtual void getObjects(std::vector<ObjectConstPtr>& objects, bool exclude_robots = true) const = 0;

    /**
         * Returns all objects stored in this world that overlap with the specified axis aligned
         * bounding box.
         * @param aabb - Axis aligned bounding box in world frame.
         * @param objects - vector to store output in
         * @param exclude_robots - if true, robots are not returned
         */
    virtual void getObjects(const BoundingBox& aabb, std::vector<ObjectPtr>& objects,
        bool exclude_robots = true)
        = 0;
    virtual void getObjects(const BoundingBox& aabb, std::vector<ObjectConstPtr>& objects,
        bool exclude_robots = true) const = 0;

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
    virtual void stepPhysics(int steps = 1) = 0;

    /**
         * If the underlying world representation supports physics simulation,
         * this function issues a physics simulation step. This version of the function
         * fills the provided list of contacts with any contacts that occurred during the propagation.
         * @param steps number of physics time steps to simulate.
         */
    virtual void stepPhysics(std::vector<Contact>& contacts, int steps = 1) = 0;

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
         * Checks whether the current state is physically feasible.
         * A state is physically feasible as long as rigid bodies are not intersecting.
         * Collisions between rigid bodies are allowed. Penetrations between rigid bodies,
         * however, is not allowed.
         * @return true / false
         */
    virtual bool isPhysicallyFeasible() = 0;

    /**
         * Clones this world. The returned world is identical to this world.
         */
    virtual WorldPtr clone() const = 0;

    /**
         * Checks whether there are any links that collide.
         * @param contacts - all contacts found
         * @return true iff there are at least two colliding links
         */
    virtual bool checkCollision(std::vector<Contact>& contacts) = 0;

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
        std::vector<Contact>& contacts)
        = 0;

    /**
         * Checks whether object_a is in collision with any of the provided objects.
         * @param object_a
         * @param other_objects
         * @param contacts - (optional) all contacts are stored in this list
         * @return true iff object_a collides with any of the provided objects in other_objects
         */
    virtual bool checkCollision(ObjectPtr object_a, const std::vector<ObjectPtr>& other_objects) = 0;
    virtual bool checkCollision(ObjectPtr object_a, const std::vector<ObjectPtr>& other_objects,
        std::vector<Contact>& contacts)
        = 0;

    /**
         * Checks whether link_a is in collision with any of the provided objects.
         * @param link_a
         * @param other_objects
         * @param contacts - (optional) all contacts are stored in this list
         * @return true iff link_a collides with any of the provided objects in other_objects
         */
    virtual bool checkCollision(LinkPtr link_a, const std::vector<ObjectPtr>& other_objects) = 0;
    virtual bool checkCollision(LinkPtr link_a, const std::vector<ObjectPtr>& other_objects,
        std::vector<Contact>& contacts)
        = 0;
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
     * Returns an instance of an image renderer.
     * The image renderer is independent of the Viewer. That is 
     * @return shared pointer to an image renderer that allows rendering images from this world
     */
    virtual WorldViewer::ImageRendererPtr getImageRenderer() = 0;

    /**
         * Returns an instance of the logger used in the context of this environment.
         * @return shared pointer to a logger.
         */
    virtual LoggerPtr getLogger() = 0;
    virtual LoggerConstPtr getConstLogger() const = 0;

    /**
         * Returns whether the world is at rest in the current state.
         * The world is at rest, if all object's velocities are below the given threshold.
         * @return true/false depending on whether the world is at rest.
         */
    virtual bool atRest(float threshold = 0.0001f) const = 0;

    /**
         * Retrieve the state of this world, i.e. all ObjectStates for all objects/robots
         */
    virtual WorldState getWorldState() const = 0;
    virtual void getWorldState(WorldState& state) const = 0;

    /**
         * Prints the current state using the world's logger.
         */
    virtual void printWorldState(Logger::LogLevel level = Logger::LogLevel::Debug) const = 0;

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
     *  Removes the last saved state without setting the world to it.
     */
    virtual void dropState() = 0;

    /**
         * Returns a mutex to lock this world.
         * Any function that changes the state of this world should lock it first.
         * @return recursive mutex for this world
         */
    virtual std::recursive_mutex& getMutex() const = 0;
};
} // namespace sim_env

#endif //SIM_ENV_H

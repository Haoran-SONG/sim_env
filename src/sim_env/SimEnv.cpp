//
// Created by joshua on 7/31/17.
//
#include "sim_env/SimEnv.h"

using namespace sim_env;

sim_env::Logger::~Logger() = default;

sim_env::Entity::~Entity() = default;

sim_env::Ball::Ball() = default;

sim_env::Ball::Ball(const Ball& other) = default;

sim_env::Ball::Ball(Eigen::Vector3f lcenter, float lradius)
    : center(lcenter)
    , radius(lradius)
{
}

sim_env::Ball::~Ball() = default;

Ball& sim_env::Ball::operator=(const Ball& other) = default;

sim_env::Link::~Link() = default;

sim_env::Joint::~Joint() = default;

sim_env::Object::~Object() = default;

sim_env::Robot::~Robot() = default;

sim_env::World::~World() = default;

sim_env::WorldViewer::~WorldViewer() = default;

std::atomic_uint sim_env::WorldViewer::Handle::_global_id_counter(1);

sim_env::WorldViewer::Handle::Handle(bool valid_handle)
{
    if (valid_handle) {
        _id = _global_id_counter++;
    } else {
        _id = 0;
    }
}

bool sim_env::WorldViewer::Handle::isValid() const
{
    return _id != 0;
}

sim_env::WorldViewer::Handle::Handle(const WorldViewer::Handle& other)
{
    _id = other._id;
}

sim_env::WorldViewer::Handle::~Handle() = default;

sim_env::WorldViewer::ImageRenderer::~ImageRenderer() = default;

WorldViewer::Handle& WorldViewer::Handle::operator=(const WorldViewer::Handle& other)
{
    _id = other._id;
    return *this;
}

unsigned int sim_env::WorldViewer::Handle::getID() const
{
    return _id;
}

void sim_env::Logger::logErr(const boost::format& bf, const std::string& prefix) const
{
    logErr(boost::str(bf), prefix);
}

void sim_env::Logger::logInfo(const boost::format& bf, const std::string& prefix) const
{
    logInfo(boost::str(bf), prefix);
}

void sim_env::Logger::logWarn(const boost::format& bf, const std::string& prefix) const
{
    logWarn(boost::str(bf), prefix);
}

void sim_env::Logger::logDebug(const boost::format& bf, const std::string& prefix) const
{
    logDebug(boost::str(bf), prefix);
}

void sim_env::Logger::log(const boost::format& bf, LogLevel level, const std::string& prefix) const
{
    log(boost::str(bf), level, prefix);
}

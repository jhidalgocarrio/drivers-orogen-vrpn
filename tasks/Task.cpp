/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <string.h>

using namespace vrpn;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    this->host = _server.value();
    RTT::log(RTT::Debug) << "[VRPN] Connecting to server: "<<this->host << RTT::endlog();
    this->connection = std::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name(this->host.c_str()));
    RTT::log(RTT::Debug) << "[VRPN] Connection stablished" << RTT::endlog();


    this->pose.sourceFrame = _source_frame.value();
    this->pose.targetFrame = _tracker_name.value();
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    this->connection->mainloop();

    RTT::log(RTT::Debug) << "[VRPN] Creating new tracker: "<<_tracker_name.value();
    this->tracker = std::make_shared<vrpn_Tracker_Remote>(_tracker_name.value().c_str(), this->connection.get());
    this->tracker->register_change_handler(this, &Task::handle_pose);
    this->tracker->register_change_handler(this, &Task::handle_twist);
    this->tracker->register_change_handler(this, &Task::handle_accel);
    this->tracker->shutup = false;
    RTT::log(RTT::Debug) << " [STABLISHED]" << RTT::endlog();

    this->valid_data = false;

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    if (!this->connection->doing_okay())
    {
        RTT::log(RTT::Debug) << "[VRPN] Connection not doing okay"<< RTT::endlog();
    }
    this->tracker->mainloop();
    int i = 0;
    while (this->connection->sender_name(i) != NULL)
    {
        if (strcmp(_tracker_name.value().c_str(), this->connection->sender_name(i)) == 0)
        {
            //std::cout<<"["<<i<<"] Name: "<<this->connection->sender_name(i)<<std::endl;
            if (this->valid_data)
            {
                this->pose.setTransform(this->pose.getTransform() * _delta_trans.value().toTransform());
                RTT::log(RTT::Info) << "[VRPN] "<<_tracker_name.value()<<" previous: "<< previous.toMilliseconds()<<" new: "<<  this->pose.time.toMilliseconds() <<" delta_t: " << (this->pose.time.toMilliseconds()-previous.toMilliseconds())/1000.0 <<RTT::endlog();
                _pose_sample.write(this->pose);
                this->previous = this->pose.time;
                this->valid_data = false;
            }
        }
        i++;
    }

}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
    RTT::log(RTT::Debug) << "[VRPN] Destroying tracker... " <<_tracker_name.value();
    this->tracker->unregister_change_handler(this, &Task::handle_pose);
    this->tracker->unregister_change_handler(this, &Task::handle_twist);
    this->tracker->unregister_change_handler(this, &Task::handle_accel);
    RTT::log(RTT::Debug) << "[DONE]"<<RTT::endlog();

}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    this->connection.reset();
}

void VRPN_CALLBACK Task::handle_pose(void *task, const vrpn_TRACKERCB tracker_pose)
{
    ::base::samples::RigidBodyState &rbs = static_cast<Task* >(task)->pose;
    base::Time ts = ::base::Time::now();
    if (((Task*)task)->_use_server_time.value())
    {
        ts = ::base::Time::fromMicroseconds(tracker_pose.msg_time.tv_sec * 1e06 + tracker_pose.msg_time.tv_usec);
        RTT::log(RTT::Debug) << "\ttime: "<< tracker_pose.msg_time.tv_sec << "." <<tracker_pose.msg_time.tv_usec <<RTT::endlog();
        RTT::log(RTT::Debug) << "\trock time: "<< ts.toString() <<RTT::endlog();
    }
    else
    {
        RTT::log(RTT::Debug) << "\trock time: "<< ts.toString() <<RTT::endlog();
    }
    RTT::log(RTT::Debug) << "\tpose: "<< tracker_pose.pos[0] << "," <<tracker_pose.pos[1] << "," <<tracker_pose.pos[2]<<RTT::endlog();
    RTT::log(RTT::Debug) << "\tquat: "<< tracker_pose.quat[0] << "," <<tracker_pose.quat[1] << "," <<tracker_pose.quat[2]<<","<<tracker_pose.quat[3]<<RTT::endlog();
    rbs.time = ts;
    rbs.position <<  tracker_pose.pos[0],  tracker_pose.pos[1],  tracker_pose.pos[2]; // x y z
    rbs.orientation = base::Quaterniond(tracker_pose.quat[3],  tracker_pose.quat[0],  tracker_pose.quat[1], tracker_pose.quat[2]); //quat is coming in w x y z , Eigen::quaterniond expects x y z w
    static_cast<Task*>(task)->valid_data = true;

}
void VRPN_CALLBACK Task::handle_twist(void *task, const vrpn_TRACKERVELCB tracker_twist)
{
    ::base::samples::RigidBodyState &rbs = static_cast<Task* >(task)->pose;
    rbs.velocity << tracker_twist.vel[0], tracker_twist.vel[1], tracker_twist.vel[2];
    RTT::log(RTT::Debug) << "\tlin_velocity: "<< tracker_twist.vel[0] << "," <<tracker_twist.vel[1] << "," <<tracker_twist.vel[2]<<RTT::endlog();
    static_cast<Task*>(task)->valid_data = true;
}

void VRPN_CALLBACK Task::handle_accel(void *data, const vrpn_TRACKERACCCB tracker_accel)
{

}


#ifndef FACTORY_GAZEBO_PLUGIN
#define FACTORY_GAZEBO_PLUGIN


#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <gazebo_ros/node.hpp>

#include <iostream>
#include <string>
#include <fstream>
#include <streambuf>
#include <mutex>    
#include <functional>

#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{

class Factory : public WorldPlugin
{
public: 
    void Load(physics::WorldPtr _parent, sdf::ElementPtr sdf);

private:
    physics::WorldPtr _world;
    std::mutex mtx; 
    gazebo::event::ConnectionPtr _update_connection;
    std::unordered_map<std::string, std::array<int, 5>> _casualties;
    std::vector <std::string> _patient_models;
    unsigned int _model_count= 0;
    bool _updated = false;
    bool _init = false;

    void get_casualty_numbers();
    void OnUpdate();
    void add_model(unsigned int& patient_number) const;

    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}

#endif  // FACTORY_GAZEBO_PLUGIN

#include "factory/factory.hpp"
// #include <ignition/math/Pose3.hh>
#include <chrono>
#include <cmath>
#include <regex>

namespace gazebo
{
using namespace std::chrono_literals;

void Factory::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{ 
    ros_node_ = gazebo_ros::Node::Get(_sdf);
    _world = _parent;

    ros_node_->declare_parameter<int>("triage/patient/P0/count", 0);
    ros_node_->declare_parameter<int>("triage/patient/P1/count", 0);
    ros_node_->declare_parameter<int>("triage/patient/P2/count", 0);
    ros_node_->declare_parameter<int>("triage/patient/P3/count", 0);

    ros_node_->declare_parameter<int>("cgh/patient/P0/count", 0);
    ros_node_->declare_parameter<int>("cgh/patient/P1/count", 0);
    ros_node_->declare_parameter<int>("cgh/patient/P2/count", 0);
    ros_node_->declare_parameter<int>("cgh/patient/P3/count", 0);

    timer_ = ros_node_->create_wall_timer(
        5s, std::bind(&Factory::get_casualty_numbers, this));

    RCLCPP_INFO(ros_node_->get_logger(), "Loading RTLS factory Gazebo Plugin");

    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&gazebo::Factory::OnUpdate, this));   
}

void Factory::get_casualty_numbers()
{
    _casualties.clear();   
    std::array<int,5> count = {0,0,0,0,0};

    ros_node_->get_parameter("cgh/patient/P0/count", count[1]);
    ros_node_->get_parameter("cgh/patient/P1/count", count[2]);
    ros_node_->get_parameter("cgh/patient/P2/count", count[3]);
    ros_node_->get_parameter("cgh/patient/P3/count", count[4]);
    for (unsigned int i=1; i< 5; i++)
    {
        if (std::signbit(count[i])) //True if negative
            continue;
        else
            count[0] = count[0] + count[i];
    }

    _casualties.insert(std::make_pair(std::string("CGH"), count)); 
    count = {0,0,0,0,0};

    ros_node_->get_parameter("triage/patient/P0/count", count[1]);
    ros_node_->get_parameter("triage/patient/P1/count", count[2]);
    ros_node_->get_parameter("triage/patient/P2/count", count[3]);
    ros_node_->get_parameter("triage/patient/P3/count", count[4]);
    for (unsigned int i=1; i< 5; i++)
    {
        if (std::signbit(count[i])) //True if negative
            continue;
        else
            count[0] = count[0] + count[i];
    }

    _casualties.insert(std::make_pair(std::string("TRIAGE"), count)); 

    for (const auto& v : _casualties)
    {
        RCLCPP_INFO(ros_node_->get_logger(), 
            "Location at %s with total casualties"
            " Total: %d P0: %d P1: %d P2: %d P3: %d", 
            v.first.c_str(), v.second[0], v.second[1],v.second[2],v.second[3],v.second[4]);
    }
    _updated = true;
}

void Factory::OnUpdate()
{
    const std::lock_guard<std::mutex> lock(mtx);

    if (_updated == false)
    {
        // Casualty numbers has not been updated.
        return;
    }
    
    // Error checking
    if (_casualties.empty())
    {
        return;
    }

    // if no patients -> do nothing
    if (_casualties.at("CGH")[0] <1)
    {
        return;
    }

    // Load initial models
    // For cgh
    if (_init == false && !_casualties.empty() && _casualties.at("CGH")[0] != -1)
    { 
        for (unsigned int i = 0; i <_casualties.at("CGH")[0]; i++)
        {
            add_model(i);
            // _model_count = _world->ModelCount();
        }
        _init = true;
        std::cout<<"Initdone"<<std::endl;
    }

    /*
    if updated
        Get list of partient models in gz

        Check _casualties numbers against patient models

        _casualties->patient count == patient model count
            Assume that nothing changed
            Continue
        
        _casualties->patient count > patient model count
            Assume that _casualties have decreased in number
            Delete model
        
        _casualties->patient count < patient model count
            Assume that _casualties have increased in number
            Generate a new model
    else
        Wait for casualty numbers to be updates
    */

    if (_updated == true)
    {
        // Get list of patients in cgh
        unsigned int total_cgh_casualties = _casualties.at("CGH")[0];

        // Get list of patient models in gz
        std::vector<int> casualty_models;
        const auto& all_models = _world->Models();
        if (all_models.empty()) 
        {
            RCLCPP_INFO(ros_node_->get_logger(), "No gz models found" ); 
            return;
        }
        else
        {
            const std::regex pieces_regex("(patient)(\\d+)");
            std::smatch pieces_match;
            for (const auto& m : all_models) // For each model
            {
                if (m == nullptr) continue;
                std::string model_name = m->GetName(); 
                std::regex_match(model_name, pieces_match, pieces_regex); 
                
                if (pieces_match[2].str().empty() != true)
                    casualty_models.emplace_back(std::stoi(pieces_match[2].str()));
            }

            if (casualty_models.empty()) return;

            std::sort(casualty_models.begin(),casualty_models.end());

            if (casualty_models.size() == total_cgh_casualties)
            {    
                std::cout<<"casualty_models.size() == total_cgh_casualties"<<std::endl;
            }
            else if (casualty_models.size() > total_cgh_casualties)
            {
                std::cout<<"casualty_models.size() > total_cgh_casualties "<< casualty_models.size()<< " " <<total_cgh_casualties<<std::endl;
                // To remove models
                casualty_models.erase(casualty_models.begin(),casualty_models.begin()+total_cgh_casualties);
                for (const auto& e : casualty_models)
                {
                    _world->RemoveModel("patient"+std::to_string(e));
                }
            }
            else if (casualty_models.size() < total_cgh_casualties)
            {
                std::cout<<"casualty_models.size() < total_cgh_casualties "<< casualty_models.size()<< " " <<total_cgh_casualties<<std::endl;
                // To add more models
                for (unsigned int i=casualty_models.size(); i<total_cgh_casualties; i++)
                {
                    add_model(i);
                }
            }
            // Processed the recently updated casualties. Resetting variable now.
            _updated = false;
        }
    }
    else
        // Wait for casualty numbers to be updated
        return;

}

void Factory::add_model(unsigned int& patient_number) const
{
    sdf::SDF patientSDF;
    patientSDF.SetFromString(
        "<sdf version ='1.4'>\
        <model name ='sphere'>\
            <pose>15.0 -20.0 0.0 0 0 0.0</pose>\
            <link name ='link'>\
            <collision name ='collision'>\
                <pose>0.0 0.0 0.3 0 0 0.0</pose>\
                <geometry>\
                <sphere><radius>0.3</radius></sphere>\
                </geometry>\
            </collision>\
            <visual name ='visual'>\
                <geometry>\
                <mesh>\
                    <uri>model://cardboard_box/meshes/cardboard_box.dae</uri>\
                </mesh>\
                </geometry>\
            </visual>\
            </link>\
            <static>False</static>\
        </model>\
        </sdf>");
    sdf::ElementPtr model = patientSDF.Root()->GetElement("model");

    std::string model_name = "patient" + std::to_string(patient_number);
    model->GetAttribute("name")->SetFromString(model_name);
    _world->InsertModelSDF(patientSDF);
}

GZ_REGISTER_WORLD_PLUGIN(Factory)
}
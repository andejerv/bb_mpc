#include <iostream>
#include <memory>
#include <vector>
#include <chrono>
#include <functional>

#include "../include/bb_mpc/SB_MPC.h"
#include "eigen3/Eigen/Dense"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "mavros_msgs/mavros_msgs/msg/global_position_target.hpp"
#include "mavros_msgs/mavros_msgs/msg/waypoint.hpp"
#include "mavros_msgs/mavros_msgs/msg/waypoint_list.hpp"
#include "mavros_msgs/mavros_msgs/srv/command_bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class mpc_node : public rclcpp::Node
{
private:

    bool active;

    Eigen::Vector2d position;
    Eigen::Vector2d target = Eigen::Vector2d(-35.5, 150);

    sbmpc::SB_MPC mpc;
    sbmpc::BB_MODEL agent;
    std::vector<sbmpc::Object*> obstacles;

    rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr control_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_signal_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_subscription_;
    rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr waypoint_subscription_;
    rclcpp::QoS pos_qos = rclcpp::QoS(rclcpp::KeepLast(10));

    sbmpc::Stopwatch sw;

public:
    mpc_node(int n_steps, double step_size) : Node("mpc_node"), active(false), mpc(n_steps, step_size), agent(0,0){
        RCLCPP_INFO(this->get_logger(), "Starting up now");

        pos_qos.best_effort();

        control_publisher_ = this->create_publisher<mavros_msgs::msg::GlobalPositionTarget>("/mavros/setpoint_raw/global", 10);
        control_timer_ = this->create_wall_timer(1000ms, std::bind(&mpc_node::control_timer_callback, this));
        run_signal_ = this->create_subscription<std_msgs::msg::Bool>("/run_mpc", 10, std::bind(&mpc_node::run_callback, this, _1));
        position_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", pos_qos, std::bind(&mpc_node::position_msg_callback, this, _1));
        waypoint_subscription_ = this->create_subscription<mavros_msgs::msg::WaypointList>("/mavros/mission/waypoints", pos_qos, std::bind(&mpc_node::waypoint_msg_callback, this, _1));
    }

    ~mpc_node(){
        RCLCPP_INFO(this->get_logger(), "Shutting down");
        clearObstacles();
    }

    void control_timer_callback(){
        if (active){
            this->sw.start();
            double obs_lat = -35.36257755;
            double obs_long = 149.1648973;
            double obs_size = 1;
            double obs_safe_dist = 20;
            double obs_x = (obs_long - this->position(0)) * 111000 * cos(this->position(1)* M_PI / 180);
            double obs_y = (obs_lat - this->position(1))*111000;


            this->obstacles.push_back(new sbmpc::Object(obs_x, obs_y, obs_size, obs_size, obs_safe_dist));

            double hdg = sbmpc::waypointToHeading(this->position, this->target);

            double hdg_os_best = 0;
            double vel_os_best = 1;

            this->mpc.getBestControlOffset(hdg_os_best, vel_os_best, hdg, 3, &this->agent, this->obstacles);

            mavros_msgs::msg::GlobalPositionTarget cmd;
            cmd.header = std_msgs::msg::Header();
            
            cmd.type_mask = 2535; //1511 = vel+yaw_rate, 2527 = vel+yaw
            cmd.coordinate_frame = 6;
            cmd.velocity.x = 3*vel_os_best;
            cmd.yaw = hdg + hdg_os_best; // counter-clockwise from east in radians
            

            control_publisher_->publish(cmd);
            std::cout << "Heading:" << cmd.yaw << std::endl;

            this->clearObstacles();
            double elapsed = this->sw.stop();
            std::cout << "Elapsed time: " << elapsed << " s" << std::endl << std::endl;
        }
    }

    void run_callback(const std_msgs::msg::Bool::SharedPtr msg){
        this->active = msg->data;
    }

    void position_msg_callback(const sensor_msgs::msg::NavSatFix & msg){
        this->position(0) = msg.longitude;
        this->position(1) = msg.latitude;
        //std::cout << this->position << std::endl << std::endl;
    }

    void waypoint_msg_callback(const mavros_msgs::msg::WaypointList & msg){
        std::cout << "Received new waypoint list" << std::endl;
        mavros_msgs::msg::Waypoint wp = msg.waypoints.at(msg.current_seq);
        this->target(0) = wp.y_long;
        this->target(1) = wp.x_lat;
        std::cout << this->target << std::endl;
    }

    void addStaticObstacle(double x, double y, double size, double collisionDist, double safeDist){
        sbmpc::Object* obstacle = new sbmpc::Object(x, y, size, collisionDist, safeDist);
        this->obstacles.push_back(obstacle);
    }

    void addDynamicObstacle(double x, double y, double size, double collisionDist, double safeDist, double vel, double hdg){
        sbmpc::Object* obstacle = new sbmpc::Object(x, y, size, collisionDist, safeDist, vel, hdg);
        obstacles.push_back(obstacle);
    }

    void clearObstacles(){
        for (auto obstacle : obstacles){
            delete obstacle;
        }
        obstacles.clear();
    }

};


int main(int argc, char * argv[]){

    std::cout << "Hello, World!" << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mpc_node>(200, 0.2));
    rclcpp::shutdown();

    return 0;
}
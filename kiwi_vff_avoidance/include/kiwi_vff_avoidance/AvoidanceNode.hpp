#ifndef KIWI_VFF_AVOIDANCE__AVOIDANCENODE_HPP
#define KIWI_VFF_AVOIDANCE__AVOIDANCENODE_HPP

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace kiwi_vff_avoidance
{
    struct VFFVectors
    {
        std::vector<float> attractive = {0.0, 0.0};
        std::vector<float> repulsive = {0.0, 0.0};
        std::vector<float> result;

        std::vector<float> get_result()
        {
            std::vector<float> ans = {attractive[0] + repulsive[0], attractive[1] + repulsive[1]};
            return ans;

        }
    };

    class AvoidanceNode : public rclcpp::Node
    {
    public:
        AvoidanceNode();
    
        void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
        void control_cycle();

    protected:
        VFFVectors get_vff(const sensor_msgs::msg::LaserScan& scan);

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vff_debug_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

    };
}

#endif
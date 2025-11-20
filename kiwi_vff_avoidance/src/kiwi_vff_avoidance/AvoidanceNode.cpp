#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "kiwi_vff_avoidance/AvoidanceNode.hpp"

using std::placeholders::_1;

namespace kiwi_vff_avoidance
{
    std::vector<float> VFFVectors::get_result()
    {
        return std::vector<float>{ attractive[0] + repulsive[0], attractive[1] + repulsive[1]};
    }

    AvoidanceNode::AvoidanceNode() : Node("kiwi_avoidance_vff")
    {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "input_scan",
            rclcpp::SensorDataQoS(),
            std::bind(&AvoidanceNode::scan_callback, this, _1)
        );

    }

    void AvoidanceNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
    {
        last_scan_ = std::move(msg);
    }
    void AvoidanceNode::control_cycle()
    {


    }

    VFFVectors AvoidanceNode::get_vff(const sensor_msgs::msg::LaserScan& scan)
    {
        const float THRESHOLD_DIST = 1.0;

        VFFVectors vff;
        vff.attractive = {THRESHOLD_DIST, 0.0};
        vff.repulsive = {0.0, 0.0};
        vff.result = {0.0, 0.0};

        auto min_ind = std::min_element(scan.ranges.begin(), scan.ranges.end()) - scan.ranges.begin();
        float min_dist = scan.ranges[min_ind];

        if (min_dist < THRESHOLD_DIST)
        {
            auto ang = scan.angle_min + min_ind * scan.angle_increment;
            auto opposite_ang = ang + M_PI;
            auto complementary_dist = THRESHOLD_DIST - min_dist;

            vff.repulsive[0] = cos(opposite_ang) * complementary_dist;
            vff.repulsive[1] = sin(opposite_ang) * complementary_dist;
        }

        vff.result = vff.get_result();
        
        return vff;
    }
}
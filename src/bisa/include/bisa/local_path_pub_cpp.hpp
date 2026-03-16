#ifndef BISA_LOCAL_PATH_PUB_CPP_HPP_
#define BISA_LOCAL_PATH_PUB_CPP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "bisa/msg/lap_info.hpp"
#include <optional>

namespace bisa
{

    class LocalPathPubCpp : public rclcpp::Node
    {
    public:
        LocalPathPubCpp();

    private:
        void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void publish_local_path();
        void publish_car_marker();

        nav_msgs::msg::Path::SharedPtr global_path_;
        std::optional<geometry_msgs::msg::Pose> current_pose_;

        size_t local_path_size_ = 220;
        size_t current_waypoint_ = 0;
        int lap_count_ = 0;

        bool is_initialized_ = false;
        int search_window = 200;
        size_t max_index_step_ = 20;
        double reinit_distance_threshold_ = 2.0;
        int smooth_window_ = 1;
        int corner_smooth_window_ = 0;
        double corner_curvature_threshold_ = 0.08;

        rclcpp::Time lap_start_time_;
        double total_distance_ = 0.0;
        std::optional<geometry_msgs::msg::Pose> prev_pose_;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

        // 기존 멤버 변수들 아래에 추가
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rviz_local_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_marker_pub_;
        int rviz_slot_ = -1; // -1이면 RViz 발행 비활성화

        rclcpp::Publisher<bisa::msg::LapInfo>::SharedPtr lap_info_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Time last_log_time_{0, 0, RCL_ROS_TIME};
    };

}

#endif

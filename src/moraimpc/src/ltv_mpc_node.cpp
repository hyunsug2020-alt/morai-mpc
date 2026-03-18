#include <ros/rospy.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <morai_msgs/CtrlCmd.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <tf/transformations.h>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <fstream>

#include "moraimpc/ltv_types.hpp"
#include "moraimpc/ltv_model.hpp"
#include "moraimpc/ltv_cost.hpp"
#include "moraimpc/ltv_solver.hpp"

namespace moraimpc {

class LTVMPCNode {
public:
    LTVMPCNode() {
        ros::NodeHandle nh;
        
        // Load params
        std::string path_file;
        nh.param<std::string>("path_file", path_file, "/home/david/recorded_path.json");
        loadPath(path_file);

        // Subscribers
        odom_sub_ = nh.subscribe("/eskf/odom", 1, &LTVMPCNode::odomCallback, this);
        
        // Publishers
        ctrl_pub_ = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 1);
        perf_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/mpc_performance", 1);
        status_pub_ = nh.advertise<std_msgs::String>("/mpc_status", 1);

        model_ = std::make_unique<LTVModel>(cfg_);
        cost_ = std::make_unique<LTVCost>(cfg_);
        solver_ = std::make_unique<LTVSolver>(cfg_);

        timer_ = nh.createTimer(ros::Duration(cfg_.Ts), &LTVMPCNode::controlLoop, this);
        
        current_kappa_ = 0.0;
        ROS_INFO("C++ LTV-MPC Node Started");
    }

private:
    void loadPath(const std::string& file) {
        std::ifstream ifs(file);
        Json::Value root;
        Json::Reader reader;
        if (reader.parse(ifs, root)) {
            const Json::Value wps = root["waypoints"];
            for (int i = 0; i < wps.size(); ++i) {
                waypoints_x_.push_back(wps[i]["x"].asDouble());
                waypoints_y_.push_back(wps[i]["y"].asDouble());
            }
        }
        ROS_INFO("Loaded %zu waypoints", waypoints_x_.size());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        cur_x_ = msg->pose.pose.position.x;
        cur_y_ = msg->pose.pose.position.y;
        cur_v_ = std::sqrt(std::pow(msg->twist.twist.linear.x, 2) + std::pow(msg->twist.twist.linear.y, 2));
        
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, cur_yaw_);
        is_odom_received_ = true;
    }

    void controlLoop(const ros::TimerEvent& event) {
        if (!is_odom_received_ || waypoints_x_.empty()) return;

        auto t_start = ros::WallTime::now();

        // 1. Find nearest waypoint and build reference
        int nearest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        for(size_t i=0; i<waypoints_x_.size(); ++i) {
            double d = std::hypot(waypoints_x_[i] - cur_x_, waypoints_y_[i] - cur_y_);
            if(d < min_dist) { min_dist = d; nearest_idx = i; }
        }

        // x0: [dr, theta, kappa, theta_r, kappa_r]
        Eigen::VectorXd x0(kNx);
        double x_ref = waypoints_x_[nearest_idx];
        double y_ref = waypoints_y_[nearest_idx];
        
        int next_idx = (nearest_idx + 1) % waypoints_x_.size();
        double theta_ref = std::atan2(waypoints_y_[next_idx] - y_ref, waypoints_x_[next_idx] - x_ref);
        
        // Lateral error dr
        double dx = cur_x_ - x_ref;
        double dy = cur_y_ - y_ref;
        double dr = -std::sin(theta_ref) * dx + std::cos(theta_ref) * dy;
        
        x0 << dr, cur_yaw_, current_kappa_, theta_ref, 0.0; // kappa_r simplified to 0

        // 2. Build Matrices
        std::vector<double> v_profile(cfg_.N, cfg_.target_vel);
        Eigen::MatrixXd A_bar, B_bar, E_bar;
        model_->buildBatchMatrices(v_profile, A_bar, B_bar, E_bar);

        Eigen::SparseMatrix<double> P;
        Eigen::VectorXd q;
        Eigen::VectorXd z_bar = Eigen::VectorXd::Zero(cfg_.N); // disturbance
        cost_->buildQPObjective(x0, z_bar, A_bar, B_bar, E_bar, P, q);

        // 3. Constraints
        Eigen::SparseMatrix<double> A_cons(cfg_.N, cfg_.N);
        A_cons.setIdentity();
        Eigen::VectorXd l_cons = Eigen::VectorXd::Constant(cfg_.N, cfg_.u_min);
        Eigen::VectorXd u_cons = Eigen::VectorXd::Constant(cfg_.N, cfg_.u_max);

        // 4. Solve
        Eigen::VectorXd solution;
        bool success = solver_->solve(P, q, A_cons, l_cons, u_cons, solution);

        auto t_end = ros::WallTime::now();
        double solve_time_ms = (t_end - t_start).toSec() * 1000.0;

        if (success && solution.size() > 0) {
            double u0 = solution[0];
            current_kappa_ += u0 * cfg_.Ts;
            current_kappa_ = std::max(cfg_.kappa_min, std::min(cfg_.kappa_max, current_kappa_));

            morai_msgs::CtrlCmd cmd;
            cmd.longlCmdType = 2;
            cmd.velocity = cfg_.target_vel * 3.6;
            // MORAI steer: - sign convention
            cmd.steering = -std::atan(current_kappa_ * cfg_.L) * (180.0 / M_PI);
            ctrl_pub_.publish(cmd);

            std_msgs::Float32MultiArray perf;
            perf.data = {(float)min_dist, (float)solve_time_ms, 0.0f};
            perf_pub_.publish(perf);
            
            std_msgs::String status;
            status.data = "OPTIMAL";
            status_pub_.publish(status);
        } else {
            ROS_WARN_THROTTLE(1.0, "MPC Solver Failed");
        }
    }

    LTVMPCConfig cfg_;
    std::unique_ptr<LTVModel> model_;
    std::unique_ptr<LTVCost> cost_;
    std::unique_ptr<LTVSolver> solver_;

    std::vector<double> waypoints_x_, waypoints_y_;
    double cur_x_, cur_y_, cur_v_, cur_yaw_, current_kappa_;
    bool is_odom_received_ = false;

    ros::Subscriber odom_sub_;
    ros::Publisher ctrl_pub_, perf_pub_, status_pub_;
    ros::Timer timer_;
};

} // namespace moraimpc

int main(int argc, char** argv) {
    ros::init(argc, argv, "ltv_mpc_node");
    moraimpc::LTVMPCNode node;
    ros::spin();
    return 0;
}

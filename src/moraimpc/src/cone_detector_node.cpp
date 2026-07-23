// 3D LiDAR Cone Detector — 2 LiDAR (앞/뒤) fusion + 자동 z 캘리브
// Pipeline:
//  - lidar_1, lidar_2 PointCloud2 subscribe
//  - 각 cloud ground plane RANSAC → z_offset 자동 추정 (running avg)
//  - 각 cloud → vehicle frame transform (x/y/yaw는 launch arg, z는 auto)
//  - 두 cloud combine → ROI/voxel/ground/cluster → 콘 필터 → map frame publish
//
// 입력: ~lidar_1, ~lidar_2 (sensor_msgs/PointCloud2), /localization/ego_status
// 출력: ~cones (PoseArray, map frame), ~cone_markers (MarkerArray)

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

class ConeDetector {
public:
    using PointT = pcl::PointXYZ;
    using CloudT = pcl::PointCloud<PointT>;

    ConeDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
        // 일반 파라미터
        pnh.param<std::string>("frame_id",  frame_id_,  "ego_vehicle");
        pnh.param<std::string>("map_frame", map_frame_, "map");
        pnh.param<std::string>("ego_topic", ego_topic_, "/localization/ego_status");
        pnh.param("publish_tf", publish_tf_, true);
        pnh.param("dedup_dist", dedup_dist_, 0.50);
        pnh.param("accumulate", accumulate_, false);
        pnh.param("confirm_min_hits", confirm_min_hits_, 3);

        // ROI (vehicle frame, 두 LiDAR fusion 후 적용)
        pnh.param("roi_x_min", roi_x_min_, -30.0);
        pnh.param("roi_x_max", roi_x_max_,  30.0);
        pnh.param("roi_y_min", roi_y_min_, -30.0);
        pnh.param("roi_y_max", roi_y_max_,  30.0);
        pnh.param("roi_z_min", roi_z_min_,  -2.5);
        pnh.param("roi_z_max", roi_z_max_,   1.0);
        pnh.param("voxel_leaf", voxel_leaf_, 0.05);

        // 차량 자체 footprint 제외 (LiDAR가 차 본체 보는 거 차단)
        pnh.param("ego_box_x_min", ego_box_xmin_, -2.5);
        pnh.param("ego_box_x_max", ego_box_xmax_,  2.5);
        pnh.param("ego_box_y_min", ego_box_ymin_, -1.1);
        pnh.param("ego_box_y_max", ego_box_ymax_,  1.1);

        pnh.param("ground_dist_thresh",   ground_dist_thresh_,   0.05);
        pnh.param("ground_max_iter",      ground_max_iter_,      200);
        pnh.param("ground_angle_eps_deg", ground_angle_eps_deg_, 15.0);

        pnh.param("cluster_tolerance", cluster_tol_, 0.30);
        pnh.param("cluster_min_size",  cluster_min_, 3);
        pnh.param("cluster_max_size",  cluster_max_, 300);

        pnh.param("cone_height_min", cone_h_min_, 0.20);
        pnh.param("cone_height_max", cone_h_max_, 0.90);
        pnh.param("cone_radius_min", cone_r_min_, 0.05);
        pnh.param("cone_radius_max", cone_r_max_, 0.40);

        pnh.param("debug_clusters", debug_clusters_, false);
        pnh.param("show_all_clusters", show_all_, false);

        // LiDAR 설정 (asdf.json 기본은 전방 3D LiDAR 한 대)
        // x/y/yaw: launch arg, z: 자동 캘리브 (ground RANSAC running avg)
        std::string topic1, topic2;
        pnh.param<std::string>("lidar1_topic", topic1, "/velodyne_points");
        pnh.param<std::string>("lidar2_topic", topic2, "/lidar3D_2");
        bool use_lidar2 = false;
        pnh.param("use_lidar2", use_lidar2, false);

        LidarSrc s1, s2;
        s1.topic = topic1;
        s2.topic = topic2;
        pnh.param("lidar1_x",   s1.x,    1.676);
        pnh.param("lidar1_y",   s1.y,    0.005);
        pnh.param("lidar1_yaw", s1.yaw,  0.0);
        pnh.param("lidar2_x",   s2.x,   -1.5);
        pnh.param("lidar2_y",   s2.y,    0.0);
        pnh.param("lidar2_yaw", s2.yaw,  M_PI);   // 후방 LiDAR는 반대 방향
        pnh.param("z_calib_n",  z_calib_n_, 30);  // ground RANSAC 평균 frame 수

        s1.idx = 0; s2.idx = 1;
        srcs_.push_back(s1);
        if (use_lidar2) srcs_.push_back(s2);

        // Subscribers
        sub_l1_ = nh.subscribe<sensor_msgs::PointCloud2>(
            srcs_[0].topic, 1, boost::bind(&ConeDetector::cloudCb, this, _1, 0));
        if (use_lidar2) {
            sub_l2_ = nh.subscribe<sensor_msgs::PointCloud2>(
                srcs_[1].topic, 1,
                boost::bind(&ConeDetector::cloudCb, this, _1, 1));
        }
        sub_ego_ = nh.subscribe(ego_topic_, 5, &ConeDetector::egoCb, this);

        pub_pose_ = pnh.advertise<geometry_msgs::PoseArray>("cones", 1);
        pub_mk_   = pnh.advertise<visualization_msgs::MarkerArray>("cone_markers", 1);
        pub_comb_ = pnh.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);
        pub_proc_ = pnh.advertise<sensor_msgs::PointCloud2>("processed_cloud", 1);

        if (use_lidar2) {
            ROS_INFO("[ConeDetector] L1=%s (x=%.3f,y=%.3f) L2=%s (x=%.3f,y=%.3f) z=auto(N=%d)",
                     srcs_[0].topic.c_str(), srcs_[0].x, srcs_[0].y,
                     srcs_[1].topic.c_str(), srcs_[1].x, srcs_[1].y,
                     z_calib_n_);
        } else {
            ROS_INFO("[ConeDetector] single LiDAR=%s (x=%.3f,y=%.3f,yaw=%.1f deg) z=auto(N=%d)",
                     srcs_[0].topic.c_str(), srcs_[0].x, srcs_[0].y,
                     srcs_[0].yaw*180.0/M_PI, z_calib_n_);
        }
    }

private:
    struct LidarSrc {
        std::string topic;
        double x, y, yaw;
        // 자동 캘리브레이션: ground RANSAC z 평균
        double z_running = 0.0;
        int    z_samples = 0;
        double z_offset = 1.5;     // 초기값, 자동 갱신
        bool   z_locked = false;
        bool   pre_transformed = false;  // PointCloud가 이미 vehicle frame이면 transform skip
        // 최신 cloud (vehicle frame)
        CloudT::Ptr latest;
        ros::Time   last_t;
        int idx = 0;
    };

    void egoCb(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
        // Low-pass filter (위치/yaw 노이즈 제거 — RViz 흔들림 방지)
        double new_x   = msg->position.x;
        double new_y   = msg->position.y;
        double new_yaw = msg->heading * M_PI / 180.0;
        const double a = 0.3;  // 0=완전 smooth, 1=raw
        if (!ego_ready_) {
            ego_x_ = new_x; ego_y_ = new_y; ego_yaw_ = new_yaw;
        } else {
            ego_x_   = (1.0 - a) * ego_x_ + a * new_x;
            ego_y_   = (1.0 - a) * ego_y_ + a * new_y;
            // yaw는 wrap 처리
            double dy = new_yaw - ego_yaw_;
            while (dy >  M_PI) dy -= 2 * M_PI;
            while (dy < -M_PI) dy += 2 * M_PI;
            ego_yaw_ += a * dy;
        }
        ego_ready_ = true;

        // TF publish 20Hz로 throttle (60Hz → 미세 노이즈 누적 방지)
        ros::Time now = ros::Time::now();
        if (publish_tf_ && (now - last_tf_t_).toSec() >= 0.05) {
            geometry_msgs::TransformStamped tf;
            tf.header.stamp = now;
            tf.header.frame_id = map_frame_;
            tf.child_frame_id  = frame_id_;
            tf.transform.translation.x = ego_x_;
            tf.transform.translation.y = ego_y_;
            tf.transform.translation.z = 0.0;
            tf2::Quaternion q; q.setRPY(0, 0, ego_yaw_);
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();
            tf_br_.sendTransform(tf);
            last_tf_t_ = now;
        }
    }

    // LiDAR cloud cb. idx=0 (lidar1) cb를 master로 사용 — fusion + detection 트리거.
    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg, int idx) {
        auto& src = srcs_[idx];
        CloudT::Ptr in(new CloudT);
        pcl::fromROSMsg(*msg, *in);
        if (in->empty()) return;

        // z 자동 추정 (지면 위치 정보용 — pre_transformed 판정)
        if (!src.z_locked) {
            double gz;
            if (estimateGroundZ(in, gz)) {
                src.z_running = (src.z_running * src.z_samples + (-gz)) / (src.z_samples + 1);
                src.z_samples++;
                src.z_offset = src.z_running;
                if (src.z_samples >= z_calib_n_) {
                    src.z_locked = true;
                    // z_offset이 작으면 (<0.3m) PointCloud가 이미 vehicle frame
                    src.pre_transformed = (std::abs(src.z_offset) < 0.30);
                    ROS_INFO("[ConeDetector] LiDAR%d 캘리브: z=%.2fm  pre_transformed=%s",
                             idx+1, src.z_offset, src.pre_transformed ? "YES (transform skip)" : "NO (transform apply)");
                }
            }
        }

        // Transform: pre_transformed면 skip (이미 vehicle frame), 아니면 x/y/yaw/z 적용
        CloudT::Ptr in_veh(new CloudT);
        if (src.pre_transformed || !src.z_locked) {
            // pre_transformed 또는 캘리브 진행 중: 그대로 사용
            in_veh = in;
        } else {
            Eigen::Affine3f T = Eigen::Affine3f::Identity();
            T.translation() << src.x, src.y, src.z_offset;
            T.rotate(Eigen::AngleAxisf(src.yaw, Eigen::Vector3f::UnitZ()));
            pcl::transformPointCloud(*in, *in_veh, T);
        }

        src.latest = in_veh;
        src.last_t = ros::Time::now();

        // 3. master (idx=0)이면 fusion + detection
        if (idx != 0) return;

        CloudT::Ptr combined(new CloudT);
        *combined += *srcs_[0].latest;
        if (srcs_.size() > 1 && srcs_[1].latest) {
            // L2 stale (>0.5s)면 현재 프레임에서 제외한다.
            bool l2_fresh =
                (ros::Time::now() - srcs_[1].last_t).toSec() < 0.5;
            if (l2_fresh) *combined += *srcs_[1].latest;
        }

        // 합쳐진 cloud publish (vehicle frame, RViz 시각화용)
        sensor_msgs::PointCloud2 comb_msg;
        pcl::toROSMsg(*combined, comb_msg);
        comb_msg.header.stamp = ros::Time::now();
        comb_msg.header.frame_id = frame_id_;
        pub_comb_.publish(comb_msg);

        runDetection(combined);
    }

    bool estimateGroundZ(const CloudT::Ptr& cloud, double& gz) {
        // RANSAC plane (수평) → 평균 z
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.10);
        seg.setAxis(Eigen::Vector3f(0, 0, 1));
        seg.setEpsAngle(15.0 * M_PI / 180.0);
        seg.setInputCloud(cloud);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
        seg.segment(*inliers, *coef);
        if (inliers->indices.size() < 50) return false;

        // 평균 z
        double sum = 0;
        for (int i : inliers->indices) sum += (*cloud)[i].z;
        gz = sum / inliers->indices.size();
        return true;
    }

    void runDetection(const CloudT::Ptr& cloud) {
        // ROI passthrough
        CloudT::Ptr roi(new CloudT);
        applyROI(cloud, roi);

        // 차량 자체 footprint 제외
        CloudT::Ptr no_ego(new CloudT);
        excludeEgoBox(roi, no_ego);

        // Voxel
        CloudT::Ptr vox(new CloudT);
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(no_ego);
        vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
        vg.filter(*vox);
        if (vox->size() < 50) {
            publishResults();
            return;
        }

        // Ground RANSAC 제거
        CloudT::Ptr no_gnd(new CloudT);
        removeGround(vox, no_gnd);
        if (no_gnd->size() < 20) {
            publishResults();
            return;
        }

        // 처리된 cloud publish (지면 제거 후 — 콘이 잘 보이는 깨끗한 cloud)
        sensor_msgs::PointCloud2 proc_msg;
        pcl::toROSMsg(*no_gnd, proc_msg);
        proc_msg.header.stamp = ros::Time::now();
        proc_msg.header.frame_id = frame_id_;
        pub_proc_.publish(proc_msg);

        // Cluster
        std::vector<pcl::PointIndices> clusters;
        clusterize(no_gnd, clusters);

        // Cone filter
        std::vector<Eigen::Vector3f> cones_local;
        int idx_dbg = 0;
        for (const auto& c : clusters) {
            Eigen::Vector3f centroid; float h, r;
            bool is_cone = isCone(no_gnd, c, centroid, h, r);
            if (debug_clusters_) {
                ROS_INFO("  cluster[%d] n=%zu h=%.2f r=%.2f pos=(%.1f,%.1f,%.1f) %s",
                         idx_dbg, c.indices.size(), h, r, centroid.x(), centroid.y(), centroid.z(),
                         is_cone ? "CONE" : "skip");
            }
            ++idx_dbg;
            if (!is_cone && !show_all_) continue;
            cones_local.push_back(centroid);
        }

        // Vehicle → map
        if (!accumulate_) cones_map_raw_.clear();
        if (ego_ready_) {
            const double cs = std::cos(ego_yaw_), sn = std::sin(ego_yaw_);
            for (const auto& cl : cones_local) {
                double mx = ego_x_ + cs * cl.x() - sn * cl.y();
                double my = ego_y_ + sn * cl.x() + cs * cl.y();
                if (accumulate_) addOrUpdateCone(mx, my, cl.z());
                else cones_map_raw_.push_back({mx, my, (double)cl.z(), confirm_min_hits_});
            }
        }
        publishResults();

        size_t confirmed = 0;
        for (const auto& c : cones_map_raw_) if (c.hits >= confirm_min_hits_) ++confirmed;
        ROS_INFO_THROTTLE(1.0,
            "[ConeDetector] L1z=%.2f%s%s L2z=%.2f%s%s comb=%zu vox=%zu noGnd=%zu cl=%zu cones=%zu",
            srcs_[0].z_offset, srcs_[0].z_locked ? "*" : "", srcs_[0].pre_transformed ? "[veh]" : "",
            srcs_[1].z_offset, srcs_[1].z_locked ? "*" : "", srcs_[1].pre_transformed ? "[veh]" : "",
            cloud->size(), vox->size(), no_gnd->size(), clusters.size(), confirmed);
    }

    void applyROI(const CloudT::Ptr& in, CloudT::Ptr& out) {
        CloudT::Ptr t1(new CloudT), t2(new CloudT);
        pcl::PassThrough<PointT> pt;
        pt.setInputCloud(in); pt.setFilterFieldName("x");
        pt.setFilterLimits(roi_x_min_, roi_x_max_); pt.filter(*t1);
        pt.setInputCloud(t1); pt.setFilterFieldName("y");
        pt.setFilterLimits(roi_y_min_, roi_y_max_); pt.filter(*t2);
        pt.setInputCloud(t2); pt.setFilterFieldName("z");
        pt.setFilterLimits(roi_z_min_, roi_z_max_); pt.filter(*out);
    }

    void excludeEgoBox(const CloudT::Ptr& in, CloudT::Ptr& out) {
        out->clear();
        out->reserve(in->size());
        for (const auto& p : *in) {
            bool inside = (p.x >= ego_box_xmin_ && p.x <= ego_box_xmax_ &&
                           p.y >= ego_box_ymin_ && p.y <= ego_box_ymax_);
            if (!inside) out->push_back(p);
        }
    }

    void removeGround(const CloudT::Ptr& in, CloudT::Ptr& out) {
        // Multi-plane RANSAC: 큰 수평 평면 3번까지 순차 제거 (도로 + 인도 + 경사면 등 다중 지형)
        *out = *in;
        const int max_passes = 3;
        const size_t min_inliers = 100;  // 이거 미만 평면이면 stop

        for (int pass = 0; pass < max_passes; ++pass) {
            if (out->size() < 50) break;
            pcl::SACSegmentation<PointT> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(ground_max_iter_);
            seg.setDistanceThreshold(ground_dist_thresh_);
            seg.setAxis(Eigen::Vector3f(0, 0, 1));
            seg.setEpsAngle(ground_angle_eps_deg_ * M_PI / 180.0);
            seg.setInputCloud(out);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
            seg.segment(*inliers, *coef);
            if (inliers->indices.size() < min_inliers) break;

            CloudT::Ptr next(new CloudT);
            pcl::ExtractIndices<PointT> ex;
            ex.setInputCloud(out); ex.setIndices(inliers); ex.setNegative(true);
            ex.filter(*next);
            *out = *next;
        }
    }

    void clusterize(const CloudT::Ptr& in, std::vector<pcl::PointIndices>& clusters) {
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(in);
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tol_);
        ec.setMinClusterSize(cluster_min_);
        ec.setMaxClusterSize(cluster_max_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(in);
        ec.extract(clusters);
    }

    bool isCone(const CloudT::Ptr& cloud, const pcl::PointIndices& idx,
                Eigen::Vector3f& centroid, float& h, float& r) {
        h = 0; r = 0;
        if (idx.indices.empty()) return false;
        Eigen::Vector4f mn, mx;
        pcl::getMinMax3D(*cloud, idx, mn, mx);
        h = mx.z() - mn.z();
        r = 0.5f * std::max(mx.x() - mn.x(), mx.y() - mn.y());
        Eigen::Vector4f c4;
        pcl::compute3DCentroid(*cloud, idx, c4);
        centroid = Eigen::Vector3f(c4.x(), c4.y(), mn.z());
        if (h < cone_h_min_ || h > cone_h_max_) return false;
        if (r < cone_r_min_ || r > cone_r_max_) return false;
        if (h < r * 1.5f) return false;  // 콘은 위로 길쭉 (h/r ≥ 1.5)
        // 콘 base가 ground 근처 (z 너무 위에 떠있으면 콘 아님 — 빌딩 일부 등)
        // mn.z()는 클러스터 최저점, ground 근처여야 함
        return true;
    }

    void addOrUpdateCone(double mx, double my, double mz) {
        for (auto& ec : cones_map_raw_) {
            if (std::hypot(ec.x - mx, ec.y - my) < dedup_dist_) {
                ec.x = 0.7 * ec.x + 0.3 * mx;
                ec.y = 0.7 * ec.y + 0.3 * my;
                ec.z = 0.7 * ec.z + 0.3 * mz;
                ec.hits++;
                return;
            }
        }
        cones_map_raw_.push_back({mx, my, mz, 1});
    }

    void publishResults() {
        geometry_msgs::PoseArray pa;
        pa.header.stamp = ros::Time::now();
        pa.header.frame_id = map_frame_;
        for (const auto& c : cones_map_raw_) {
            if (c.hits < confirm_min_hits_) continue;
            geometry_msgs::Pose p;
            p.position.x = c.x; p.position.y = c.y; p.position.z = c.z;
            p.orientation.w = 1.0;
            pa.poses.push_back(p);
        }
        pub_pose_.publish(pa);

        visualization_msgs::MarkerArray ma;
        visualization_msgs::Marker del;
        del.header = pa.header;
        del.action = visualization_msgs::Marker::DELETEALL;
        ma.markers.push_back(del);
        int mid = 0;
        for (const auto& c : cones_map_raw_) {
            if (c.hits < confirm_min_hits_) continue;
            visualization_msgs::Marker m;
            m.header = pa.header;
            m.ns = "cones"; m.id = mid++;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = c.x;
            m.pose.position.y = c.y;
            m.pose.position.z = c.z + 0.25;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.30; m.scale.y = 0.30; m.scale.z = 0.50;
            m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f; m.color.a = 0.95f;
            m.lifetime = ros::Duration(0);
            ma.markers.push_back(m);
        }
        pub_mk_.publish(ma);
    }

    // 일반 cfg
    std::string frame_id_, map_frame_, ego_topic_;
    double dedup_dist_;
    bool   accumulate_, publish_tf_, debug_clusters_, show_all_;
    int    confirm_min_hits_;
    int    z_calib_n_;

    // ROI
    double roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_;
    double voxel_leaf_;
    double ego_box_xmin_, ego_box_xmax_, ego_box_ymin_, ego_box_ymax_;
    double ground_dist_thresh_, ground_angle_eps_deg_;
    int    ground_max_iter_;
    double cluster_tol_;
    int    cluster_min_, cluster_max_;
    double cone_h_min_, cone_h_max_, cone_r_min_, cone_r_max_;

    // Ego
    double ego_x_ = 0, ego_y_ = 0, ego_yaw_ = 0;
    bool   ego_ready_ = false;
    ros::Time last_tf_t_;

    // 누적 콘
    struct ConeRec { double x, y, z; int hits; };
    std::vector<ConeRec> cones_map_raw_;

    // LiDAR sources (2개)
    std::vector<LidarSrc> srcs_;

    ros::Subscriber sub_l1_, sub_l2_, sub_ego_;
    ros::Publisher  pub_pose_, pub_mk_, pub_comb_, pub_proc_;
    tf2_ros::TransformBroadcaster tf_br_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cone_detector_node");
    ros::NodeHandle nh, pnh("~");
    ConeDetector det(nh, pnh);
    ros::spin();
    return 0;
}

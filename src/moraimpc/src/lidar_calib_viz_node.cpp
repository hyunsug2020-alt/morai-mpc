// LiDAR 캘리브레이션 검증 시각화 (OpenCV 2D top-down)
// 목적: 차량 전방 + 2 LiDAR 데이터 방향 일치 확인
//
// 표시:
//   - 차량 외곽 (4.6m × 1.9m, IONIQ 5)
//   - 전방 화살표 (붉은색)
//   - LiDAR 1 점 (파랑)
//   - LiDAR 2 점 (주황)
//   - LiDAR 마운트 위치 (큰 점)
//   - 그리드 (1m 간격)
//
// 입력: /lidar3D_1, /lidar3D_2 (sensor_msgs/PointCloud2)
// 출력: OpenCV 창 "Lidar Calibration"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <mutex>
#include <vector>

class LidarCalibViz {
public:
    using PointT = pcl::PointXYZ;
    using CloudT = pcl::PointCloud<PointT>;

    LidarCalibViz(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
        // 파라미터
        std::string topic1, topic2;
        pnh.param<std::string>("lidar1_topic", topic1, "/lidar3D_1");
        pnh.param<std::string>("lidar2_topic", topic2, "/lidar3D_2");
        pnh.param("view_range_m", view_range_m_, 25.0);     // 표시 반경 ±25m
        pnh.param("img_size",     img_size_,     900);       // 창 크기 (px)
        pnh.param("vehicle_length", veh_L_, 4.635);
        pnh.param("vehicle_width",  veh_W_, 1.892);
        pnh.param("lidar1_x", l1_x_,  1.5);
        pnh.param("lidar1_y", l1_y_,  0.0);
        pnh.param("lidar2_x", l2_x_, -1.5);
        pnh.param("lidar2_y", l2_y_,  0.0);
        pnh.param("z_min",    z_min_, -1.5);
        pnh.param("z_max",    z_max_,  1.0);

        sub1_ = nh.subscribe<sensor_msgs::PointCloud2>(
            topic1, 1, [this](const sensor_msgs::PointCloud2ConstPtr& m){ this->cloudCb(m, 0); });
        sub2_ = nh.subscribe<sensor_msgs::PointCloud2>(
            topic2, 1, [this](const sensor_msgs::PointCloud2ConstPtr& m){ this->cloudCb(m, 1); });

        ROS_INFO("[LidarCalibViz] L1=%s L2=%s view=±%.1fm img=%d",
                 topic1.c_str(), topic2.c_str(), view_range_m_, img_size_);

        cv::namedWindow(kWin, cv::WINDOW_AUTOSIZE);
        cv::moveWindow(kWin, 100, 100);

        // 메인 루프 (15Hz draw)
        timer_ = nh.createTimer(ros::Duration(1.0/15.0), &LidarCalibViz::draw, this);
    }

    ~LidarCalibViz() {
        cv::destroyWindow(kWin);
    }

private:
    static constexpr const char* kWin = "Lidar Calibration";

    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg, int idx) {
        CloudT::Ptr c(new CloudT);
        pcl::fromROSMsg(*msg, *c);
        std::lock_guard<std::mutex> lk(mtx_);
        clouds_[idx] = c;
    }

    cv::Point2i toPixel(double x, double y) const {
        // vehicle frame (x=앞, y=좌) → 이미지 좌표 (px=오른쪽, py=아래)
        // 차량 앞이 위쪽이 되도록 회전: img_x = -y, img_y = -x
        double sx = (img_size_ * 0.5) / view_range_m_;
        int px = static_cast<int>(img_size_ / 2.0 - y * sx);
        int py = static_cast<int>(img_size_ / 2.0 - x * sx);
        return cv::Point2i(px, py);
    }

    void drawGrid(cv::Mat& img) {
        const int step_m = 5;
        cv::Scalar minor(40, 40, 45);
        cv::Scalar major(70, 70, 75);
        // m grid
        for (int m = -((int)view_range_m_); m <= (int)view_range_m_; ++m) {
            cv::Scalar c = (m % step_m == 0) ? major : minor;
            cv::line(img, toPixel((double)m, -view_range_m_), toPixel((double)m, view_range_m_), c, 1);
            cv::line(img, toPixel(-view_range_m_, (double)m), toPixel(view_range_m_, (double)m), c, 1);
        }
        // 축 (vehicle frame x=앞, y=좌)
        cv::line(img, toPixel(0, 0), toPixel(view_range_m_, 0), cv::Scalar(40, 40, 200), 2);  // 앞=빨강
        cv::line(img, toPixel(0, 0), toPixel(0, view_range_m_), cv::Scalar(40, 200, 40), 2);  // 좌=초록
    }

    void drawVehicle(cv::Mat& img) {
        // 차량 직사각형 (4.6×1.9, vehicle frame: x=길이방향, y=폭방향)
        std::vector<cv::Point2i> pts;
        pts.push_back(toPixel( veh_L_/2,  veh_W_/2));
        pts.push_back(toPixel( veh_L_/2, -veh_W_/2));
        pts.push_back(toPixel(-veh_L_/2, -veh_W_/2));
        pts.push_back(toPixel(-veh_L_/2,  veh_W_/2));
        cv::polylines(img, pts, true, cv::Scalar(255, 255, 255), 2);

        // 전방 화살표 (앞쪽 강조)
        cv::arrowedLine(img, toPixel(0, 0), toPixel(veh_L_/2 + 1.5, 0),
                        cv::Scalar(40, 40, 255), 3, cv::LINE_AA, 0, 0.15);

        // 앞 마크 (헤드라이트 위치 표시)
        cv::circle(img, toPixel(veh_L_/2, veh_W_/2 - 0.3), 5, cv::Scalar(0, 255, 255), -1);
        cv::circle(img, toPixel(veh_L_/2, -veh_W_/2 + 0.3), 5, cv::Scalar(0, 255, 255), -1);
    }

    void drawLidarMounts(cv::Mat& img) {
        // LiDAR 1 (파랑)
        cv::circle(img, toPixel(l1_x_, l1_y_), 8, cv::Scalar(255, 200, 50), -1);
        cv::circle(img, toPixel(l1_x_, l1_y_), 10, cv::Scalar(255, 255, 255), 2);
        cv::putText(img, "L1", toPixel(l1_x_ + 0.5, l1_y_ + 0.5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        // LiDAR 2 (주황)
        cv::circle(img, toPixel(l2_x_, l2_y_), 8, cv::Scalar(50, 165, 255), -1);
        cv::circle(img, toPixel(l2_x_, l2_y_), 10, cv::Scalar(255, 255, 255), 2);
        cv::putText(img, "L2", toPixel(l2_x_ + 0.5, l2_y_ + 0.5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    void drawCloud(cv::Mat& img, CloudT::Ptr cloud, cv::Scalar color) {
        if (!cloud) return;
        for (const auto& p : *cloud) {
            if (p.z < z_min_ || p.z > z_max_) continue;
            if (std::abs(p.x) > view_range_m_ || std::abs(p.y) > view_range_m_) continue;
            cv::Point2i pt = toPixel(p.x, p.y);
            if (pt.x < 0 || pt.x >= img_size_ || pt.y < 0 || pt.y >= img_size_) continue;
            img.at<cv::Vec3b>(pt.y, pt.x) = cv::Vec3b((uchar)color[0], (uchar)color[1], (uchar)color[2]);
        }
    }

    void drawHUD(cv::Mat& img) {
        cv::rectangle(img, cv::Rect(0, 0, 320, 100), cv::Scalar(0, 0, 0), -1);
        cv::rectangle(img, cv::Rect(0, 0, 320, 100), cv::Scalar(80, 80, 80), 1);
        cv::putText(img, "Lidar Calibration View",
                    cv::Point(10, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

        std::lock_guard<std::mutex> lk(mtx_);
        char buf[128];
        size_t n1 = clouds_[0] ? clouds_[0]->size() : 0;
        size_t n2 = clouds_[1] ? clouds_[1]->size() : 0;
        snprintf(buf, sizeof(buf), "L1 (blue):   %zu pts", n1);
        cv::putText(img, buf, cv::Point(10, 48), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(255, 200, 50), 1);
        snprintf(buf, sizeof(buf), "L2 (orange): %zu pts", n2);
        cv::putText(img, buf, cv::Point(10, 68), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(50, 165, 255), 1);
        snprintf(buf, sizeof(buf), "Front->RED arrow  (X+ axis)");
        cv::putText(img, buf, cv::Point(10, 88), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(40, 40, 255), 1);

        // 우측 상단: 범례
        int rx = img_size_ - 200, ry = 10;
        cv::rectangle(img, cv::Rect(rx, ry, 190, 90), cv::Scalar(0, 0, 0), -1);
        cv::rectangle(img, cv::Rect(rx, ry, 190, 90), cv::Scalar(80, 80, 80), 1);
        cv::putText(img, "Legend", cv::Point(rx + 8, ry + 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        cv::circle(img, cv::Point(rx + 18, ry + 38), 5, cv::Scalar(255, 200, 50), -1);
        cv::putText(img, "L1 mount (blue)", cv::Point(rx + 30, ry + 42),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 200, 50), 1);
        cv::circle(img, cv::Point(rx + 18, ry + 58), 5, cv::Scalar(50, 165, 255), -1);
        cv::putText(img, "L2 mount (orange)", cv::Point(rx + 30, ry + 62),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(50, 165, 255), 1);
        cv::line(img, cv::Point(rx + 12, ry + 78), cv::Point(rx + 25, ry + 78),
                 cv::Scalar(40, 40, 255), 2);
        cv::putText(img, "Vehicle FRONT (X+)", cv::Point(rx + 30, ry + 82),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(40, 40, 255), 1);
    }

    void draw(const ros::TimerEvent&) {
        cv::Mat img(img_size_, img_size_, CV_8UC3, cv::Scalar(15, 15, 18));
        drawGrid(img);

        // 점운 (vehicle frame이라 가정 — pre_transformed 모드)
        CloudT::Ptr c1, c2;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            c1 = clouds_[0]; c2 = clouds_[1];
        }
        drawCloud(img, c1, cv::Scalar(255, 200, 50));   // 파랑 — L1
        drawCloud(img, c2, cv::Scalar(50, 165, 255));   // 주황 — L2

        drawVehicle(img);
        drawLidarMounts(img);
        drawHUD(img);

        cv::imshow(kWin, img);
        cv::waitKey(1);
    }

    double view_range_m_;
    int    img_size_;
    double veh_L_, veh_W_;
    double l1_x_, l1_y_, l2_x_, l2_y_;
    double z_min_, z_max_;

    std::mutex mtx_;
    CloudT::Ptr clouds_[2];

    ros::Subscriber sub1_, sub2_;
    ros::Timer timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_calib_viz_node");
    ros::NodeHandle nh, pnh("~");
    LidarCalibViz viz(nh, pnh);
    ros::spin();
    return 0;
}

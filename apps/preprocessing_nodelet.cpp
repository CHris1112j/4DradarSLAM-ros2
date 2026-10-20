// SPDX-License-Identifier: BSD-2-Clause

#include <string>
#include <fstream>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/filter.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>

#include "radar_ego_velocity_estimator.h"
#include "rio_utils/radar_point_cloud.h"
#include "utility_radar.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>
using namespace std;

namespace radar_graph_slam {

class PreprocessingNodelet : public rclcpp::Node, public ParamServer {
public: 
  // typedef pcl::PointXYZI PointT;
  typedef pcl::PointXYZI PointT;

  PreprocessingNodelet()
    : PreprocessingNodelet(rclcpp::NodeOptions()) {}

  explicit PreprocessingNodelet(const rclcpp::NodeOptions & options)
    : rclcpp::Node("preprocessing_nodelet", options)
  {
    // ParamServer initialisieren NACH Node-Konstruktion
    ParamServer::operator=(ParamServer(this));
    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        [this]() {
            this->init_timer_->cancel(); // Timer stoppen, wir brauchen ihn nur einmal
            this->onInit();              // Jetzt ist der Knoten bereit für shared_from_this()
        }
    );
  }
  virtual ~PreprocessingNodelet() {}

  virtual void onInit() {
    // Parameter deklarieren & einlesen (ROS 2 Syntax)
    this->declare_parameter<std::string>("topic_twist",    "/eagle_data/twist");
    this->declare_parameter<std::string>("topic_inlier_pc2",  "/eagle_data/inlier_pc2");
    this->declare_parameter<std::string>("topic_outlier_pc2", "/eagle_data/outlier_pc2");
    this->declare_parameter<bool> ("enable_dynamic_object_removal", false);
    this->declare_parameter<float>("power_threshold", 0);

    std::string topic_twist      = this->get_parameter("topic_twist").as_string();
    std::string topic_inlier_pc2 = this->get_parameter("topic_inlier_pc2").as_string();
    std::string topic_outlier_pc2= this->get_parameter("topic_outlier_pc2").as_string();
    enable_dynamic_object_removal= this->get_parameter("enable_dynamic_object_removal").as_bool();
    //power_threshold              = (float)this->get_parameter("power_threshold").as_double();

    initializeTransformation();
    initializeParams();

    if(pointCloudTopic.empty()) pointCloudTopic = "/imaging_radar1";
    if(imuTopic.empty())        imuTopic = "/imu";

    auto radar_qos = rclcpp::QoS(rclcpp::KeepLast(64));
    points_sub  = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointCloudTopic, radar_qos,
        std::bind(&PreprocessingNodelet::cloud_callback, this, std::placeholders::_1));
    imu_sub     = this->create_subscription<sensor_msgs::msg::Imu>(
        imuTopic, 1,
        std::bind(&PreprocessingNodelet::imu_callback, this, std::placeholders::_1));
    command_sub = this->create_subscription<std_msgs::msg::String>(
        "/command", 10,
        std::bind(&PreprocessingNodelet::command_callback, this, std::placeholders::_1));

    // TF2 – shared_from_this() ist hier sicher (außerhalb Konstruktor)
    tf_buffer      = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());

    points_pub    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 32);
    colored_pub   = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colored_points", 32);
    imu_pub       = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 32);
    gt_pub        = this->create_publisher<nav_msgs::msg::Odometry>("/aftmapped_to_init", 16);
    pub_twist     = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(topic_twist, 5);
    pub_inlier_pc2  = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_inlier_pc2, 5);
    pub_outlier_pc2 = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_outlier_pc2, 5);
    pc2_raw_pub   = this->create_publisher<sensor_msgs::msg::PointCloud2>("/eagle_data/pc2_raw", 1);
}

private:
  void initializeTransformation(){
    livox_to_RGB = (cv::Mat_<double>(4,4) << 
    -0.006878330000, -0.999969000000, 0.003857230000, 0.029164500000,  
    -7.737180000000E-05, -0.003856790000, -0.999993000000, 0.045695200000,
     0.999976000000, -0.006878580000, -5.084110000000E-05, -0.19018000000,
    0,  0,  0,  1);
    RGB_to_livox =livox_to_RGB.inv();
    Thermal_to_RGB = (cv::Mat_<double>(4,4) <<
    0.9999526089706319, 0.008963747151337641, -0.003798822163962599, 0.18106962419014,  
    -0.008945181135788245, 0.9999481006917174, 0.004876439015823288, -0.04546324090016857,
    0.00384233617405678, -0.004842226763999368, 0.999980894463835, 0.08046453079998771,
    0,0,0,1);
    Radar_to_Thermal = (cv::Mat_<double>(4,4) <<
    0.999665,    0.00925436,  -0.0241851,  -0.0248342,
    -0.00826999, 0.999146,    0.0404891,   0.0958317,
    0.0245392,   -0.0402755,  0.998887,    0.0268037,
    0,  0,  0,  1);
    Change_Radarframe=(cv::Mat_<double>(4,4) <<
    0,-1,0,0,
    0,0,-1,0,
    1,0,0,0,
    0,0,0,1);
    Radar_to_livox=RGB_to_livox*Thermal_to_RGB*Radar_to_Thermal*Change_Radarframe;
    std::cout << "Radar_to_livox = "<< std::endl << " "  << Radar_to_livox << std::endl << std::endl;
  }
  void initializeParams() {
      // Alle Parameter deklarieren
      this->declare_parameter<std::string>("downsample_method",       "VOXELGRID");
      this->declare_parameter<double>     ("downsample_resolution",   0.05);
      this->declare_parameter<std::string>("outlier_removal_method",  "STATISTICAL");
      this->declare_parameter<int>        ("statistical_mean_k",      20);
      this->declare_parameter<double>     ("statistical_stddev",      0.2);
      this->declare_parameter<double>     ("radius_radius",           0.8);
      this->declare_parameter<int>        ("radius_min_neighbors",    2);
      this->declare_parameter<bool>       ("use_distance_filter",     true);
      this->declare_parameter<double>     ("distance_near_thresh",    1.0);
      this->declare_parameter<double>     ("distance_far_thresh",     100.0);
      this->declare_parameter<double>     ("z_low_thresh",           -5.0);
      this->declare_parameter<double>     ("z_high_thresh",           20.0);
      this->declare_parameter<std::string>("gt_file_location",        "");
      this->declare_parameter<bool>       ("publish_tf",              false);
      this->declare_parameter<double>("scan_period", 0.1);

      // Einlesen
      std::string downsample_method     = this->get_parameter("downsample_method").as_string();
      double      downsample_resolution = this->get_parameter("downsample_resolution").as_double();

      if(downsample_method == "VOXELGRID") {
          std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
          auto voxelgrid = new pcl::VoxelGrid<PointT>();
          voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
          downsample_filter.reset(voxelgrid);
      } else if(downsample_method == "APPROX_VOXELGRID") {
          std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
          pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
          approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
          downsample_filter = approx_voxelgrid;
      } else {
          if(downsample_method != "NONE") {
              std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
              std::cerr << "       : use passthrough filter" << std::endl;
          }
          std::cout << "downsample: NONE" << std::endl;
      }

      std::string outlier_removal_method = this->get_parameter("outlier_removal_method").as_string();
      if(outlier_removal_method == "STATISTICAL") {
          int    mean_k           = this->get_parameter("statistical_mean_k").as_int();
          double stddev_mul_thresh= this->get_parameter("statistical_stddev").as_double();
          std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;
          pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
          sor->setMeanK(mean_k);
          sor->setStddevMulThresh(stddev_mul_thresh);
          outlier_removal_filter = sor;
      } else if(outlier_removal_method == "RADIUS") {
          double radius        = this->get_parameter("radius_radius").as_double();
          int    min_neighbors = this->get_parameter("radius_min_neighbors").as_int();
          std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;
          pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
          rad->setRadiusSearch(radius);
          rad->setMinNeighborsInRadius(min_neighbors);
          outlier_removal_filter = rad;
       }
      // else if(outlier_removal_method == "BILATERAL") {
      //     double sigma_s = this->get_parameter("bilateral_sigma_s").as_double();
      //     double sigma_r = this->get_parameter("bilateral_sigma_r").as_double();
      //     std::cout << "outlier_removal: BILATERAL " << sigma_s << " - " << sigma_r << std::endl;
      //     pcl::FastBilateralFilter<PointT>::Ptr bilateral(new pcl::FastBilateralFilter<PointT>());
      //     fbf->setSigmaS(sigma_s);
      //     fbf->setSigmaR(sigma_r);
      //     outlier_removal_filter = bilateral;
      // } 
      else {
          std::cout << "outlier_removal: NONE" << std::endl;
      }

      use_distance_filter  = this->get_parameter("use_distance_filter").as_bool();
      distance_near_thresh = this->get_parameter("distance_near_thresh").as_double();
      distance_far_thresh  = this->get_parameter("distance_far_thresh").as_double();
      z_low_thresh         = this->get_parameter("z_low_thresh").as_double();
      z_high_thresh        = this->get_parameter("z_high_thresh").as_double();

      std::string file_name = this->get_parameter("gt_file_location").as_string();
      publish_tf            = this->get_parameter("publish_tf").as_bool();

      // GT-Datei einlesen (Logik bleibt identisch, nur ROS 2 Typen)
      std::ifstream file_in(file_name);
      if (!file_in.is_open()) {
          std::cout << "Cannot open gt file" << std::endl;
      } else {
          std::vector<std::string> vectorLines;
          std::string line;
          while (std::getline(file_in, line)) {
              vectorLines.push_back(line);
          }
          for (size_t i = 1; i < vectorLines.size(); i++) {
              double stamp, tx, ty, tz, qx, qy, qz, qw;
              std::stringstream data(vectorLines[i]);
              data >> stamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

              nav_msgs::msg::Odometry odom_msg;
              odom_msg.header.frame_id    = mapFrame;
              odom_msg.child_frame_id     = baselinkFrame;
              odom_msg.header.stamp       = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
              odom_msg.pose.pose.orientation.w = qw;
              odom_msg.pose.pose.orientation.x = qx;
              odom_msg.pose.pose.orientation.y = qy;
              odom_msg.pose.pose.orientation.z = qz;
              odom_msg.pose.pose.position.x    = tx;
              odom_msg.pose.pose.position.y    = ty;
              odom_msg.pose.pose.position.z    = tz;

              std::lock_guard<std::mutex> lock(odom_queue_mutex);
              odom_msgs.push_back(odom_msg);
          }
      }
      file_in.close();
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) {
    sensor_msgs::msg::Imu imu_data;
    imu_data.header.stamp = imu_msg->header.stamp;
    imu_data.header.frame_id = "imu_frame";
    Eigen::Quaterniond q_ahrs(imu_msg->orientation.w,
                              imu_msg->orientation.x,
                              imu_msg->orientation.y,
                              imu_msg->orientation.z);
    Eigen::Quaterniond q_r = 
        Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_rr = 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_out =  q_r * q_ahrs * q_rr;
    imu_data.orientation.w = q_out.w();
    imu_data.orientation.x = q_out.x();
    imu_data.orientation.y = q_out.y();
    imu_data.orientation.z = q_out.z();
    imu_data.angular_velocity.x = imu_msg->angular_velocity.x;
    imu_data.angular_velocity.y = -imu_msg->angular_velocity.y;
    imu_data.angular_velocity.z = -imu_msg->angular_velocity.z;
    imu_data.linear_acceleration.x = imu_msg->linear_acceleration.x;
    imu_data.linear_acceleration.y = -imu_msg->linear_acceleration.y;
    imu_data.linear_acceleration.z = -imu_msg->linear_acceleration.z;
    imu_pub->publish(imu_data);
    // imu_queue.push_back(imu_msg);
    double time_now = rclcpp::Time(imu_msg->header.stamp).seconds();
    bool updated = false;
    if (odom_msgs.size() != 0) {
      while (rclcpp::Time(odom_msgs.front().header.stamp).seconds() + 0.001 < time_now) {
        std::lock_guard<std::mutex> lock(odom_queue_mutex);
        odom_msgs.pop_front();
        updated = true;
        if (odom_msgs.size() == 0)
          break;
      }
    }
    if (updated == true && odom_msgs.size() > 0){
      if (publish_tf) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.child_frame_id = baselinkFrame;
        tf_msg.header.frame_id = mapFrame;
        tf_msg.header.stamp = odom_msgs.front().header.stamp;
        // tf_msg.header.stamp = rclcpp::Time().now();
        tf_msg.transform.rotation = odom_msgs.front().pose.pose.orientation;
        tf_msg.transform.translation.x = odom_msgs.front().pose.pose.position.x;
        tf_msg.transform.translation.y = odom_msgs.front().pose.pose.position.y;
        tf_msg.transform.translation.z = odom_msgs.front().pose.pose.position.z;
        if (tf_broadcaster) tf_broadcaster->sendTransform(tf_msg);
      }
      
      gt_pub->publish(odom_msgs.front());
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& eagle_msg)
  {
      RadarPointCloudType radarpoint_raw;
      PointT radarpoint_xyzi;
      pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw(new pcl::PointCloud<RadarPointCloudType>);
      pcl::PointCloud<PointT>::Ptr radarcloud_xyzi(new pcl::PointCloud<PointT>);

      radarcloud_xyzi->header.frame_id = baselinkFrame;
      // radarcloud_xyzi->header.seq = eagle_msg->header.seq;
      radarcloud_xyzi->header.stamp = (eagle_msg->header.stamp.sec * 1000000ull) + (eagle_msg->header.stamp.nanosec / 1000ull);
      // radarcloud_xyzi->header.stamp = static_cast<uint64_t>(rclcpp::Time(eagle_msg->header.stamp).nanoseconds() / 1e3);

      const float MIN_SNR = 13.0;

      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*eagle_msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*eagle_msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*eagle_msg, "z");
      sensor_msgs::PointCloud2ConstIterator<float> iter_snr(*eagle_msg, "snr");
      sensor_msgs::PointCloud2ConstIterator<float> iter_doppler(*eagle_msg, "velocity");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_snr, ++iter_doppler)
      {
          if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) continue;
          if (std::isnan(*iter_x)    || std::isnan(*iter_y)    || std::isnan(*iter_z))    continue;
          if (*iter_snr < MIN_SNR) continue;

          cv::Mat ptMat, dstMat;
          ptMat = (cv::Mat_<double>(4, 1) << *iter_x, *iter_y, *iter_z, 1);
          dstMat = Radar_to_livox * ptMat;

          radarpoint_raw.x         = dstMat.at<double>(0, 0);
          radarpoint_raw.y         = dstMat.at<double>(1, 0);
          radarpoint_raw.z         = dstMat.at<double>(2, 0);
          radarpoint_raw.intensity = *iter_snr;
          radarpoint_raw.doppler   = *iter_doppler;

          radarpoint_xyzi.x         = dstMat.at<double>(0, 0);
          radarpoint_xyzi.y         = dstMat.at<double>(1, 0);
          radarpoint_xyzi.z         = dstMat.at<double>(2, 0);
          radarpoint_xyzi.intensity = *iter_snr;

          radarcloud_raw->points.push_back(radarpoint_raw);
          radarcloud_xyzi->points.push_back(radarpoint_xyzi);
      }

      // ********** Publish PointCloud2 Format Raw Cloud **********
      sensor_msgs::msg::PointCloud2 pc2_raw_msg;
      pcl::toROSMsg(*radarcloud_raw, pc2_raw_msg);
      pc2_raw_msg.header.stamp    = eagle_msg->header.stamp;   // builtin_interfaces::msg::Time
      pc2_raw_msg.header.frame_id = baselinkFrame;
      pc2_raw_pub->publish(pc2_raw_msg);

      // ********** Ego Velocity Estimation **********
      Eigen::Vector3d v_r, sigma_v_r;
      sensor_msgs::msg::PointCloud2 inlier_radar_msg, outlier_radar_msg;

      auto start_ms = std::chrono::steady_clock::now();

      if (estimator.estimate(pc2_raw_msg, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg))
      {
          auto end_ms    = std::chrono::steady_clock::now();
          double time_used =
              std::chrono::duration<double>(end_ms - start_ms).count();
          egovel_time.push_back(time_used);

          geometry_msgs::msg::TwistWithCovarianceStamped twist;
          twist.header.stamp          = pc2_raw_msg.header.stamp;
          twist.twist.twist.linear.x  = v_r.x();
          twist.twist.twist.linear.y  = v_r.y();
          twist.twist.twist.linear.z  = v_r.z();

          twist.twist.covariance.at(0)  = std::pow(sigma_v_r.x(), 2);
          twist.twist.covariance.at(7)  = std::pow(sigma_v_r.y(), 2);
          twist.twist.covariance.at(14) = std::pow(sigma_v_r.z(), 2);

          pub_twist->publish(twist);
          pub_inlier_pc2->publish(inlier_radar_msg);
          pub_outlier_pc2->publish(outlier_radar_msg);
      }
      else{;}

      pcl::PointCloud<PointT>::Ptr radarcloud_inlier(new pcl::PointCloud<PointT>);
      pcl::fromROSMsg(inlier_radar_msg, *radarcloud_inlier);

      pcl::PointCloud<PointT>::ConstPtr src_cloud;
      if (enable_dynamic_object_removal)
          src_cloud = radarcloud_inlier;
      else
          src_cloud = radarcloud_xyzi;

      if(src_cloud->empty()) {
        return;
      }

      src_cloud = deskewing(src_cloud);

    // if baselinkFrame is defined, transform the input cloud to the frame
      if (!baselinkFrame.empty())
        {
          const std::string& source_frame = src_cloud->header.frame_id;

          if (!tf_buffer->canTransform(baselinkFrame, source_frame, tf2::TimePointZero))
        {
            RCLCPP_ERROR(this->get_logger(),
                "failed to find transform between %s and %s",
                baselinkFrame.c_str(), source_frame.c_str());
        }
        else
        {
            geometry_msgs::msg::TransformStamped transform_stamped;
            try {
                transform_stamped = tf_buffer->lookupTransform(
                    baselinkFrame, source_frame,
                    tf2::TimePointZero,
                    tf2::durationFromSec(2.0));
            }
            catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "TF2 lookup failed: %s", ex.what());
                return;
            }

              sensor_msgs::msg::PointCloud2 src_cloud_msg, transformed_msg;
              pcl::toROSMsg(*src_cloud, src_cloud_msg);
              src_cloud_msg.header.frame_id = source_frame;

              // tf2::doTransform für PointCloud2
              tf2::doTransform(src_cloud_msg, transformed_msg, transform_stamped);
              transformed_msg.header.frame_id = baselinkFrame;
              transformed_msg.header.stamp    = src_cloud_msg.header.stamp;

              pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>);
              pcl::fromROSMsg(transformed_msg, *transformed);
              transformed->header.frame_id = baselinkFrame;
              transformed->header.stamp    = src_cloud->header.stamp;
              src_cloud = transformed;
          }
      }

      pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(src_cloud);
      filtered = downsample(filtered);
      filtered = outlier_removal(filtered);

      // ********** Distance Histogram **********
      static size_t num_frame = 0;
      if (num_frame % 10 == 0)
      {
          Eigen::VectorXi num_at_dist = Eigen::VectorXi::Zero(100);
          for (size_t i = 0; i < filtered->size(); ++i)
          {
              int dist = static_cast<int>(std::floor(filtered->at(i).getVector3fMap().norm()));
              if (dist < 100)
                  num_at_dist(dist) += 1;
          }
          num_at_dist_vec.push_back(num_at_dist);
      }
      // ++num_frame;

      // sensor_msgs::msg::PointCloud2 filtered;
      // pcl::toROSMsg(*filtered, filtered);
      // filtered.header.stamp    = eagle_msg->header.stamp;
      // filtered.header.frame_id = baselinkFrame;
      // points_pub->publish(*filtered);
      sensor_msgs::msg::PointCloud2 ros_filtered_msg;
      pcl::toROSMsg(*filtered, ros_filtered_msg);
      ros_filtered_msg.header.stamp = eagle_msg->header.stamp;
      points_pub->publish(ros_filtered_msg);
  }


  pcl::PointCloud<PointT>::ConstPtr passthrough(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    PointT pt;
    for(int i = 0; i < cloud->size(); i++){
      if (cloud->at(i).z < 10 && cloud->at(i).z > -2){
        pt.x = (*cloud)[i].x;
        pt.y = (*cloud)[i].y;
        pt.z = (*cloud)[i].z;
        pt.intensity = (*cloud)[i].intensity;
        filtered->points.push_back(pt);
      }
    }
    filtered->header = cloud->header;
    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      // Remove NaN/Inf points
      pcl::PointCloud<PointT>::Ptr cloudout(new pcl::PointCloud<PointT>());
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud, *cloudout, indices);
      
      return cloudout;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!outlier_removal_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr distance_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());

    filtered->reserve(cloud->size());
    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const PointT& p) {
      double d = p.getVector3fMap().norm();
      double z = p.z;
      return d > distance_near_thresh && d < distance_far_thresh && z < z_high_thresh && z > z_low_thresh;
    });
    // for (size_t i=0; i<cloud->size(); i++){
    //   const PointT p = cloud->points.at(i);
    //   double d = p.getVector3fMap().norm();
    //   double z = p.z;
    //   if (d > distance_near_thresh && d < distance_far_thresh && z < z_high_thresh && z > z_low_thresh)
    //     filtered->points.push_back(p);
    // }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr deskewing(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    rclcpp::Time stamp = pcl_conversions::fromPCL(cloud->header.stamp);

    if(imu_queue.empty()) {
      return cloud;
    }

    // the color encodes the point number in the point sequence
    if(colored_pub->get_subscription_count() > 0) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>());
        colored->header = cloud->header;
        colored->is_dense = cloud->is_dense;
        colored->width = cloud->width;
        colored->height = cloud->height;
        colored->resize(cloud->size());

        for(int i = 0; i < cloud->size(); i++) {
          double t = static_cast<double>(i) / cloud->size();
          colored->at(i).getVector4fMap() = cloud->at(i).getVector4fMap();
          colored->at(i).r = 255 * t;
          colored->at(i).g = 128;
          colored->at(i).b = 255 * (1 - t);
        }
        // sensor_msgs::msg::PointCloud2 colored_msg;
        // pcl::toROSMsg(*colored, colored_msg);
        // // colored_msg.header = colored->header;
        // colored_msg.header.stamp = rclcpp::Time(colored->header.stamp * 1000);
        // colored_msg.header.frame_id = colored->header.frame_id;
        // colored_pub->publish(colored_msg);
        // colored_pub->publish(*colored);
        sensor_msgs::msg::PointCloud2 ros_colored_msg;
        pcl::toROSMsg(*colored, ros_colored_msg);
        ros_colored_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(colored->header.stamp) * 1000);
        colored_pub->publish(ros_colored_msg);
      
    }

    sensor_msgs::msg::Imu::ConstSharedPtr imu_msg = imu_queue.front();

    auto loc = imu_queue.begin();
    for(; loc != imu_queue.end(); loc++) {
      imu_msg = (*loc);
      rclcpp::Time imu_time((*loc)->header.stamp);
      if(imu_time > stamp) {
        break;
      }
    }

    imu_queue.erase(imu_queue.begin(), loc);

    Eigen::Vector3f ang_v(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    ang_v *= -1;

    pcl::PointCloud<PointT>::Ptr deskewed(new pcl::PointCloud<PointT>());
    deskewed->header = cloud->header;
    deskewed->is_dense = cloud->is_dense;
    deskewed->width = cloud->width;
    deskewed->height = cloud->height;
    deskewed->resize(cloud->size());

    double scan_period = this->get_parameter_or("scan_period", 0.1);
    for(int i = 0; i < cloud->size(); i++) {
      const auto& pt = cloud->at(i);

      // TODO: transform IMU data into the LIDAR frame
      // double t_factor = (cloud->size() > 1) ? static_cast<double>(i) / (cloud->size() - 1) : 0.0;
      // double delta_t = scan_period * t_factor;
      double delta_t = scan_period * static_cast<double>(i) / cloud->size();

      
      // Create quaternion safely
      Eigen::Quaternionf delta_q(1, delta_t / 2.0 * ang_v[0], delta_t / 2.0 * ang_v[1], delta_t / 2.0 * ang_v[2]);
      Eigen::Vector3f pt_ = delta_q.inverse() * pt.getVector3fMap();

      deskewed->at(i) = cloud->at(i);
      deskewed->at(i).getVector3fMap() = pt_;
    }

    return deskewed;
  }

  bool RadarRaw2PointCloudXYZ(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudxyz)
  {
      pcl::PointXYZ point_xyz;
      for(int i = 0; i < raw->size(); i++)
      {
          point_xyz.x = (*raw)[i].x;
          point_xyz.y = (*raw)[i].y;
          point_xyz.z = (*raw)[i].z;
          cloudxyz->points.push_back(point_xyz);
      }
      return true;
  }
  bool RadarRaw2PointCloudXYZI(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudxyzi)
  {
      pcl::PointXYZI radarpoint_xyzi;
      for(int i = 0; i < raw->size(); i++)
      {
          radarpoint_xyzi.x = (*raw)[i].x;
          radarpoint_xyzi.y = (*raw)[i].y;
          radarpoint_xyzi.z = (*raw)[i].z;
          radarpoint_xyzi.intensity = (*raw)[i].intensity;
          cloudxyzi->points.push_back(radarpoint_xyzi);
      }
      return true;
  }

  bool hasPointField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & field_name) const {
    for (const auto & field : cloud.fields) {
      if (field.name == field_name) {
        return true;
      }
    }
    return false;
  }

  void command_callback(const std_msgs::msg::String& str_msg) {
    if (str_msg.data == "time") {
      std::sort(egovel_time.begin(), egovel_time.end());
      double median = egovel_time.at(size_t(egovel_time.size() / 2));
      cout << "Ego velocity time cost (median): " << median << endl;
    }
    else if (str_msg.data == "point_distribution") {
      Eigen::VectorXi data(100);
      for (size_t i = 0; i < num_at_dist_vec.size(); i++){ // N
        Eigen::VectorXi& nad = num_at_dist_vec.at(i);
        for (int j = 0; j< 100; j++){
          data(j) += nad(j);
        }
      }
      data /= num_at_dist_vec.size();
      for (int i=0; i<data.size(); i++){
        cout << data(i) << ", ";
      }
      cout << endl;
    }
  }

private:
  // rclcpp::NodeHandle nh;
  // rclcpp::NodeHandle private_nh;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_queue;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_pub;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;


  bool use_distance_filter;
  double distance_near_thresh;
  double distance_far_thresh;
  double z_low_thresh;
  double z_high_thresh;

  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Filter<PointT>::Ptr outlier_removal_filter;

  cv::Mat Radar_to_livox; // Transform Radar point cloud to LiDAR Frame
  cv::Mat Thermal_to_RGB,Radar_to_Thermal,RGB_to_livox,livox_to_RGB,Change_Radarframe;
  rio::RadarEgoVelocityEstimator estimator;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_inlier_pc2;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_outlier_pc2;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_raw_pub;

  float power_threshold;
  bool enable_dynamic_object_removal = false;

  std::mutex odom_queue_mutex;
  std::deque<nav_msgs::msg::Odometry> odom_msgs;
  bool publish_tf;

  std::vector<double> egovel_time;

  std::vector<Eigen::VectorXi> num_at_dist_vec;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

}  // namespace radar_graph_slam

RCLCPP_COMPONENTS_REGISTER_NODE(radar_graph_slam::PreprocessingNodelet)

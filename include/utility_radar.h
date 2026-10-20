#pragma once
#ifndef _UTILITY_RADAR_ODOMETRY_H_
#define _UTILITY_RADAR_ODOMETRY_H_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// #include <opencv/cv.h>
// #include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

namespace rclcpp {

class NodeHandle
{
public:
  NodeHandle() = default;
  explicit NodeHandle(const rclcpp::Node::SharedPtr &node) : node_(node.get()) {}
  explicit NodeHandle(rclcpp::Node* node) : node_(node) {}
  template<typename T>
  T param(const std::string &name, const T &default_value) const {
    if (!node_) {
      return default_value;
    }
    if (!node_->has_parameter(name)) {
      try {
        node_->declare_parameter<T>(name, default_value);
      } catch(...) {
      }
      return default_value;
    }
    T value;
    if (node_->get_parameter(name, value)) {
      return value;
    }
    return default_value;
  }
  template<typename T>
  void param(const std::string &name, T &value, const T &default_value) const {
    value = param<T>(name, default_value);
  }
  template<typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr advertise(const std::string &topic, size_t qos) const {
    if (!node_) {
      return nullptr;
    }
    return node_->create_publisher<MsgT>(topic, rclcpp::QoS(rclcpp::KeepLast(qos)));
  }
  template<typename MsgT, typename CallbackT>
  typename rclcpp::Subscription<MsgT>::SharedPtr subscribe(const std::string &topic, size_t qos, CallbackT callback) const {
    if (!node_) {
      return nullptr;
    }
    return node_->create_subscription<MsgT>(topic, rclcpp::QoS(rclcpp::KeepLast(qos)), callback);
  }
  operator rclcpp::Node*() const {
    return node_;
  }
  rclcpp::Node* get_node() const {
    return node_;
  }
private:
  rclcpp::Node* node_ = nullptr;
};

} // namespace rclcpp

class ParamServer
{
public:

    rclcpp::NodeHandle nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Sensor Configuration:
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    void init(rclcpp::Node* node) {
        nh = rclcpp::NodeHandle(node);
        pointCloudTopic = "/radar_enhanced_pcl";
        nh.param<std::string>("pointCloudTopic", pointCloudTopic, pointCloudTopic);
        nh.param<std::string>("radar_slam/lidarFrame", lidarFrame, "livox");
        nh.param<std::string>("radar_slam/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("radar_slam/odometryFrame", odometryFrame, "odom_imu");
        nh.param<std::string>("radar_slam/mapFrame", mapFrame, "map");
        nh.param<std::string>("radar_slam/imuTopic", imuTopic, "/vectornav/imu");
        nh.param<std::string>("radar_slam/odomTopic", odomTopic, "/odom");
        nh.param<std::string>("radar_slam/gpsTopic", gpsTopic, "/ublox/fix");
        nh.param<bool>("radar_slam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("radar_slam/gpsCovThreshold", gpsCovThreshold, 2.0f);
        nh.param<float>("radar_slam/poseCovThreshold", poseCovThreshold, 25.0f);
        nh.param<bool>("radar_slam/savePCD", savePCD, false);
        nh.param<std::string>("radar_slam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");
        nh.param<int>("radar_slam/downsampleRate", downsampleRate, 1);
        nh.param<float>("radar_slam/lidarMinRange", lidarMinRange, 1.0f);
        nh.param<float>("radar_slam/lidarMaxRange", lidarMaxRange, 1000.0f);
        nh.param<float>("radar_slam/imuAccNoise", imuAccNoise, 0.01f);
        nh.param<float>("radar_slam/imuGyrNoise", imuGyrNoise, 0.001f);
        nh.param<float>("radar_slam/imuAccBiasN", imuAccBiasN, 0.0002f);
        nh.param<float>("radar_slam/imuGyrBiasN", imuGyrBiasN, 0.00003f);
        nh.param<float>("radar_slam/imuGravity", imuGravity, 9.80511f);
        nh.param<float>("radar_slam/imuRPYWeight", imuRPYWeight, 0.01f);
        nh.param<vector<double>>("radar_slam/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("radar_slam/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("radar_slam/extrinsicTrans", extTransV, vector<double>());
        if(extRotV.size()==9) extRot = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(extRotV.data(),3,3);
        if(extRPYV.size()==9) extRPY = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(extRPYV.data(),3,3);
        if(extTransV.size()==3) extTrans = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(extTransV.data(),3,1);
        extQRPY = Eigen::Quaterniond(extRPY);
    }
    ParamServer(rclcpp::Node* node = nullptr)
    {
        // Set defaults first
        pointCloudTopic = "/imaging_radar1";
        imuTopic = "/imu";
        odomTopic = "/odom";
        gpsTopic = "/ublox/fix";
        lidarFrame = "";
        baselinkFrame = "base_link";
        odometryFrame = "odom";
        mapFrame = "map";
        pointCloudTopic = "/imaging_radar1";
        imuTopic = "/imu";
        odomTopic = "/odom";
        gpsTopic = "/ublox/fix";
        lidarFrame = "";
        baselinkFrame = "base_link";
        odometryFrame = "odom";
        mapFrame = "map";
        if(node) { nh = rclcpp::NodeHandle(node); return; }
        pointCloudTopic = "/radar_enhanced_pcl";
        imuTopic = "/vectornav/imu";
        odomTopic = "/odom";
        gpsTopic = "/ublox/fix";
        lidarFrame = "livox";
        baselinkFrame = "base_link";
        odometryFrame = "odom_imu";
        mapFrame = "map";
        useGpsElevation = false;
        gpsCovThreshold = 2.0;
        poseCovThreshold = 25.0;
        savePCD = false;
        savePCDDirectory = "/Downloads/LOAM/";
        downsampleRate = 1;
        lidarMinRange = 1.0;
        lidarMaxRange = 1000.0;
        imuAccNoise = 0.01;
        imuGyrNoise = 0.001;
        imuAccBiasN = 0.0002;
        imuGyrBiasN = 0.00003;
        imuGravity = 9.80511;
        imuRPYWeight = 0.01;
        extRotV = {1,0,0, 0,1,0, 0,0,1};
        extRPYV = {1,0,0, 0,1,0, 0,0,1};
        extTransV = {0,0,0};
        extRot = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(extRotV.data(),3,3);
        extRPY = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(extRPYV.data(),3,3);
        extTrans = Eigen::Map<const Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(extTransV.data(),3,1);
        extQRPY = Eigen::Quaterniond(extRPY);
        odometrySurfLeafSize = 0.2;
        mappingCornerLeafSize = 0.2;
        mappingSurfLeafSize = 0.2;
        numberOfCores = 2;
        mappingProcessInterval = 0.15;
        surroundingkeyframeAddingDistThreshold = 1.0;
        surroundingkeyframeAddingAngleThreshold = 0.2;
        surroundingKeyframeDensity = 1.0;
        surroundingKeyframeSearchRadius = 50.0;
        loopClosureEnableFlag = false;
        loopClosureFrequency = 1.0;
        surroundingKeyframeSize = 50;
        historyKeyframeSearchRadius = 10.0;
        historyKeyframeSearchTimeDiff = 30.0;
        historyKeyframeSearchNum = 25;
        historyKeyframeFitnessScore = 0.3;
        globalMapVisualizationSearchRadius = 1e3;
        globalMapVisualizationPoseDensity = 10.0;
        globalMapVisualizationLeafSize = 1.0;
        if(!node) return;
        // Now read parameters
        nh.param<std::string>("/robot_id", robot_id, "nuc_handcart");

        pointCloudTopic = "/radar_enhanced_pcl";
        nh.param<std::string>("pointCloudTopic", pointCloudTopic, pointCloudTopic);
        nh.param<std::string>("radar_slam/pointCloudTopic", pointCloudTopic, pointCloudTopic);
        nh.param<std::string>("radar_slam/imuTopic", imuTopic, "/vectornav/imu");
        nh.param<std::string>("radar_slam/odomTopic", odomTopic, "/odom");
        nh.param<std::string>("radar_slam/gpsTopic", gpsTopic, "/ublox/fix");

        nh.param<std::string>("radar_slam/lidarFrame", lidarFrame, "livox");
        nh.param<std::string>("radar_slam/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("radar_slam/odometryFrame", odometryFrame, "odom_imu"); 
        nh.param<std::string>("radar_slam/mapFrame", mapFrame, "map");

        nh.param<bool>("radar_slam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("radar_slam/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("radar_slam/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>("radar_slam/savePCD", savePCD, false);
        nh.param<std::string>("radar_slam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        nh.param<int>("radar_slam/downsampleRate", downsampleRate, 1);
        nh.param<float>("radar_slam/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("radar_slam/lidarMaxRange", lidarMaxRange, 1000.0);

        nh.param<float>("radar_slam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("radar_slam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("radar_slam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("radar_slam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("radar_slam/imuGravity", imuGravity, 9.80511);
        nh.param<float>("radar_slam/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double>>("radar_slam/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("radar_slam/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("radar_slam/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        nh.param<float>("radar_slam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("radar_slam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("radar_slam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<int>("radar_slam/numberOfCores", numberOfCores, 2);
        nh.param<double>("radar_slam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("radar_slam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("radar_slam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("radar_slam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("radar_slam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("radar_slam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("radar_slam/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("radar_slam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("radar_slam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("radar_slam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("radar_slam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("radar_slam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("radar_slam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("radar_slam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("radar_slam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("utility_radar"), "Invalid quaternion, please use a 9-axis IMU!");
            rclcpp::shutdown();
        }

        return imu_out;
    }

};





inline sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, rclcpp::Time thisStamp, const std::string &thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub && thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return rclcpp::Time(msg->header.stamp).seconds();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


inline float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


inline float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif
#include <stdio.h>

#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp> // Zwingend nötig für ROS 2 Bag Deserialisierung
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <radar_graph_slam/graph_slam.hpp>

using namespace std;
using namespace radar_graph_slam;

std::vector<std::pair<int,int>> associate(std::vector<double> first_stamps, std::vector<double> second_stamps, double offset, double max_difference)
{
    /*
    associate timestamps

    first_stamps, second_stamps: list of timestamps to associate

    Output:
    sorted list of matches (match_first_idx, match_second_idx)
    */
    // potential_matches = [(abs(a - (b + offset)), idx_a, idx_b)
    //                      for idx_a, a in enumerate(first_stamps)
    //                      for idx_b, b in enumerate(second_stamps)
    //                      if abs(a - (b + offset)) < max_difference]
    std::vector<std::pair<int,int>> potential_matches;
    for (size_t i = 0; i < first_stamps.size(); i++){
        for (size_t j = 0; j < second_stamps.size(); j++){
            const double& a = first_stamps.at(i);
            const double& b = second_stamps.at(j);
            if (abs(a - (b + offset)) < max_difference)
                potential_matches.push_back(std::pair<int,int>(i,j));
        }
    }
    
    std::sort(potential_matches.begin(), potential_matches.end());
    // potential_matches.sort()  // prefer the closest
#if 0
    std::vector<int> first_idxes, second_idxes;
    for (int i=0; i<first_stamps.size(); i++)
        first_idxes.push_back(i);
    for (int i=0; i<second_stamps.size(); i++)
        second_idxes.push_back(i);
    std::vector<std::pair<int,int>> matches;
    
    for (int i=0; i < potential_matches.size(); i++){
        int idx_a = potential_matches.at(i).first;
        int idx_b = potential_matches.at(i).second;
        vector<int>::iterator ret_a, ret_b;
        ret_a = std::find(first_idxes.begin(), first_idxes.end(), idx_a);
        ret_b = std::find(second_idxes.begin(), second_idxes.end(), idx_b);

        bool idx_a_in_first_idxes, idx_b_in_second_idxes;
        if(ret_a == first_idxes.end()) idx_a_in_first_idxes = false;
        else idx_a_in_first_idxes = true;
        if(ret_b == second_idxes.end()) idx_b_in_second_idxes = false;
        else idx_b_in_second_idxes = true;
        
        if (idx_a_in_first_idxes && idx_b_in_second_idxes) {
            cout << first_idxes.size() << " " << idx_a << endl;
            cout << second_idxes.size() << " " << idx_b << endl;
            first_idxes.erase(first_idxes.begin()+idx_a, first_idxes.begin()+idx_a+1);
            second_idxes.erase(second_idxes.begin()+idx_b, second_idxes.begin()+idx_b+1);
            matches.push_back(std::pair<int,int>(idx_a, idx_b));
        }
    }
    
    std::sort(matches.begin(), matches.end());
    // for diff, idx_a, idx_b in potential_matches:
    //     if idx_a in first_idxes and idx_b in second_idxes:
    //         first_idxes.remove(idx_a)
    //         second_idxes.remove(idx_b)
    //         matches.append((int(idx_a), int(idx_b)))
    // matches.sort()
    return matches;
#endif
    return potential_matches;
}

// Kein rosbag::View->rosbag2_cpp::Reader
// Kein .instantiate<T>()
// sensor_msgs::NavSatFix::Ptr -> sensor_msgs::msg::NavSatFix::SharedPtr

void read_topic_from_rosbag(const std::string& bag_path, std::vector<sensor_msgs::msg::NavSatFix::SharedPtr>& msgs_queue) {
    rosbag2_cpp::Reader reader;
    reader.open(bag_path);

    rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serialization;

    while (reader.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();
        
        if (msg->topic_name == "/ublox/fix") {
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            auto navsat_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
            serialization.deserialize_message(&serialized_msg, navsat_msg.get());
            
            msgs_queue.push_back(navsat_msg);
        }
    }
}

int main()
{
    ifstream file_in("/home/zhuge/datasets/Radar/gt_loop3.txt");
    if (!file_in.is_open()) {
        cout << "Can not open the groundtruth file" << endl;
        return 0;
    }

    std::vector<sensor_msgs::msg::NavSatFix::SharedPtr> navsat_msgs;

    // In ROS 2 iterieren wir direkt über die Dateipfade
    read_topic_from_rosbag("/home/zhuge/datasets/Radar/ntu_loop3_car/ntu-car-day-loop3_2022-06-03_0.bag", navsat_msgs);
    read_topic_from_rosbag("/home/zhuge/datasets/Radar/ntu_loop3_car/ntu-car-day-loop3_2022-06-03_1.bag", navsat_msgs);
    read_topic_from_rosbag("/home/zhuge/datasets/Radar/ntu_loop3_car/ntu-car-day-loop3_2022-06-03_2.bag", navsat_msgs);

    cout << "The Number of navsat message in bag is " << navsat_msgs.size() << endl;

    std::vector<Eigen::Vector3d> utm_coordinates;
    std::vector<double> utm_stamps;
    std::vector<Eigen::Vector3d> utm_covariences;
    for (size_t i = 0; i < navsat_msgs.size(); i++) {
        if (navsat_msgs.at(i)->position_covariance.at(0) > 3 || navsat_msgs.at(i)->position_covariance.at(8) > 8)
                continue;
                
        auto gps_msg = std::make_shared<geographic_msgs::msg::GeoPointStamped>();
        gps_msg->header = navsat_msgs.at(i)->header;
        gps_msg->position.latitude = navsat_msgs.at(i)->latitude;
        gps_msg->position.longitude = navsat_msgs.at(i)->longitude;
        gps_msg->position.altitude = navsat_msgs.at(i)->altitude;
        // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
        geodesy::UTMPoint utm;
        geodesy::fromMsg(gps_msg->position, utm); 
        Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);
        Eigen::Vector3d cov(navsat_msgs.at(i)->position_covariance.at(0), navsat_msgs.at(i)->position_covariance.at(4), navsat_msgs.at(i)->position_covariance.at(8));
        utm_covariences.push_back(cov);
        utm_coordinates.push_back(xyz);
        double msg_time_sec = rclcpp::Time(navsat_msgs.at(i)->header.stamp).seconds();
        utm_stamps.push_back(msg_time_sec);
    }
    cout << "The Number of chosen utm coordinate points is " << utm_coordinates.size() << endl;
    
    std::vector<std::string> vectorLines;
    std::string line;
    while (getline(file_in, line)) 
        vectorLines.push_back(line);

    std::vector<Eigen::Vector3d> gt_coordinates;
    std::vector<double> gt_stamps;
    for (size_t i = 1; i < vectorLines.size(); i++) {
        std::string line_ = vectorLines.at(i);
        double stamp,tx,ty,tz,qx,qy,qz,qw;
        stringstream data(line_);
        data >> stamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Vector3d point(Eigen::Vector3d(tx,ty,tz));
        gt_coordinates.push_back(point);
        gt_stamps.push_back(stamp);
    }
    cout << "The Number of groundtruth points is " << gt_coordinates.size() << endl;

    vector<pair<int,int>> associated_pairs = associate(gt_stamps, utm_stamps, 0, 0.02);
    cout << "The Number of associated point pairs is " << associated_pairs.size() << endl;
    // cout << "associated_pairs:" << endl;
    // for (auto p:associated_pairs){
    //     cout << p.first << " " << p.second << endl;
    // }

    // // Calculate the initial guess, to save computation
    // int pair_size = associated_pairs.size();
    // Eigen::Matrix4Xd A (4, pair_size); // we have transposed A here for the convenience of computation
    // Eigen::Matrix4Xd b (4, pair_size);
    // for (int k = 0; k < pair_size; ++k) {
    //   A.block<3, 1>(0, k) = gt_coordinates.at(associated_pairs.at(k).first);
    //   A(3, k) = 1;
    //   b.block<3, 1>(0, k) = utm_coordinates.at(associated_pairs.at(k).second);
    //   b(3, k) = 1;
    // }
    // // solve for T
    // Eigen::Matrix4d result = A * b.inverse();
    // Eigen::Matrix3d R_0 = result.topLeftCorner(3,3);
    // Eigen::AngleAxisd aa(R_0);    // RotationMatrix to AxisAngle
    // cout << aa.axis() << endl;
    // cout << aa.axis().norm() << endl;
    // cout << aa.angle() << endl;
    // Eigen::Matrix3d R = aa.toRotationMatrix();  // AxisAngle      to RotationMatrix
    // Eigen::Vector3d t = result.block<3,1>(0,3);
    // Eigen::Matrix4d initial_transform = Eigen::Matrix4d::Identity();
    // initial_transform.topLeftCorner(3,3) = R;
    // initial_transform.block<3,1>(0,3) = t;
    // cout << "The guess transform matrix is :" << endl;
    // cout << initial_transform << endl;

    /**** Construct the pose graph ****/
    std::unique_ptr<GraphSLAM> graph_slam;
    graph_slam.reset(new GraphSLAM("lm_var"));
    
    g2o::VertexSE3* node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());

    for (size_t i = 0; i < associated_pairs.size(); i++) {
        Eigen::Vector3d& gt = gt_coordinates.at(associated_pairs.at(i).first);
        Eigen::Vector3d& utm = utm_coordinates.at(associated_pairs.at(i).second);
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
        information_matrix(0,0) /= utm_covariences.at(i)(0);
        information_matrix(1,1) /= utm_covariences.at(i)(1);
        information_matrix(2,2) /= utm_covariences.at(i)(2);
        auto edge = graph_slam->add_se3_gt_utm_edge(node, gt, utm, information_matrix);
    }
    
    // optimize the pose graph
    int num_iterations = 10240;
    graph_slam->optimize(num_iterations);
    cout << "optimized !" << endl;

    Eigen::Matrix4d transform = node->estimate().matrix();
    cout << "The final transform matrix form UTM to World is :" << endl;
    cout << fixed << setprecision(6);
    cout << transform << endl;
    
    return 0;
}
#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
import re
import os
import sys
import struct
import numpy
import scipy.io

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time

# ROS 2 bag und Punktwolken Bibliotheken
import rosbag2_py
import sensor_msgs_py.point_cloud2 as pc2
import progressbar

from sensor_msgs.msg import NavSatFix, NavSatStatus, PointCloud2
from geographic_msgs.msg import GeoPointStamped


def sec_to_time_msg(sec_float):
    """Hilfsfunktion: Konvertiert float-Sekunden in ROS 2 Time Nachricht"""
    msg = Time()
    msg.sec = int(sec_float)
    msg.nanosec = int((sec_float - msg.sec) * 1e9)
    return msg


def gps2navsat(filename, writer):
    # 1. Topics im ROS 2 Writer registrieren
    writer.create_topic(rosbag2_py.TopicMetadata(
        name='/gps/fix',
        type='sensor_msgs/msg/NavSatFix',
        serialization_format='cdr'
    ))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name='/gps/geopoint',
        type='geographic_msgs/msg/GeoPointStamped',
        serialization_format='cdr'
    ))

    # Python 3 benötigt 'rb' (read binary) für struct.unpack
    with open(filename, 'rb') as file:
        try:
            while True:
                buf1 = file.read(8*4)
                if not buf1 or len(buf1) < 32:
                    break
                data = struct.unpack('qddd', buf1)
                time_val = data[0]
                # local = data[1:] # wird nicht genutzt
                
                buf2 = file.read(8*4)
                if not buf2: break
                lat_lon_el_theta = struct.unpack('dddd', buf2)
                
                buf3 = file.read(8*16)
                if not buf3: break
                gps_cov = struct.unpack('dddddddddddddddd', buf3)

                if abs(lat_lon_el_theta[0]) < 1e-1:
                    continue

                stamp_sec = time_val * 1e-6
                stamp_msg = sec_to_time_msg(stamp_sec)
                stamp_nanos = int(stamp_sec * 1e9) # ROS 2 bag.write verlangt nanosekunden als int

                navsat = NavSatFix()
                navsat.header.frame_id = 'gps'
                navsat.header.stamp = stamp_msg
                navsat.status.status = NavSatStatus.STATUS_FIX
                navsat.status.service = NavSatStatus.SERVICE_GPS

                navsat.latitude = lat_lon_el_theta[0]
                navsat.longitude = lat_lon_el_theta[1]
                navsat.altitude = lat_lon_el_theta[2]

                navsat.position_covariance = numpy.array(gps_cov).reshape(4, 4)[:3, :3].flatten().tolist()
                navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN

                # 2. Nachricht serialisieren und schreiben
                writer.write('/gps/fix', serialize_message(navsat), stamp_nanos)

                geopoint = GeoPointStamped()
                geopoint.header = navsat.header
                geopoint.position.latitude = lat_lon_el_theta[0]
                geopoint.position.longitude = lat_lon_el_theta[1]
                geopoint.position.altitude = lat_lon_el_theta[2]

                writer.write('/gps/geopoint', serialize_message(geopoint), stamp_nanos)

        except Exception as e:
            print(f'Done reading GPS (or caught error: {e})')


def mat2pointcloud(filename):
    m = scipy.io.loadmat(filename)
    scan = numpy.transpose(m['SCAN']['XYZ'][0][0]).astype(numpy.float32)
    stamp_sec = m['SCAN']['timestamp_laser'][0][0][0][0] * 1e-6

    cloud = PointCloud2()
    cloud.header.stamp = sec_to_time_msg(stamp_sec)
    cloud.header.frame_id = 'velodyne'
    # Die pc2 API aus sensor_msgs_py funktioniert hier wie gewohnt
    cloud = pc2.create_cloud_xyz32(cloud.header, scan)
    
    return cloud, int(stamp_sec * 1e9)


def main():
    if len(sys.argv) < 2:
        print('usage: ford2bag.py output_dirname')
        return

    output_filename = sys.argv[1]

    rclpy.init()
    node = rclpy.create_node('bag_creator_node')

    filenames = sorted(['SCANS/' + x for x in os.listdir('SCANS') if re.match(r'Scan[0-9]*\.mat', x)])
    print(filenames)

    progress = progressbar.ProgressBar(max_value=len(filenames))
    pub = node.create_publisher(PointCloud2, '/velodyne_points', 32)

    # ROS 2 Sequential Writer initialisieren
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=output_filename, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    writer.open(storage_options, converter_options)
    
    writer.create_topic(rosbag2_py.TopicMetadata(
        name='/velodyne_points',
        type='sensor_msgs/msg/PointCloud2',
        serialization_format='cdr'
    ))

    gps2navsat('GPS.log', writer)
    
    for i, filename in enumerate(filenames):
        if not rclpy.ok():
            break
        progress.update(i)
        
        cloud, stamp_nanos = mat2pointcloud(filename)
        
        # In ROS 2 heißt das get_subscription_count()
        if pub.get_subscription_count() > 0:
            pub.publish(cloud)
            
        writer.write('/velodyne_points', serialize_message(cloud), stamp_nanos)

    # Aufräumen
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
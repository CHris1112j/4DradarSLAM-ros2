#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
import sys
import os
import yaml
import time
import curses
import argparse
from io import StringIO

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message, SerializationFormatConverter

# ROS 2 bag Bibliotheken
import rosbag2_py

# Nachrichten-Typen importieren
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from progressbar import ProgressBar


class BagPlayer(Node):
    def __init__(self, bagfile, start, duration):
        super().__init__('bag_player')
        
        print('Opening...')
        self.bagfile = bagfile
        
        # ROS 2 Reader und Optionen aufsetzen
        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bagfile, storage_id='sqlite3') # oder 'mcap' falls genutzt
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.reader.open(storage_options, converter_options)
        
        # YAML Info für Metadaten auslesen (ROS 2 Pfad-Struktur beachten)
        # ROS 2 Bags sind Ordner. Wir suchen nach der metadata.yaml
        metadata_path = os.path.join(bagfile, 'metadata.yaml')
        if os.path.exists(metadata_path):
            with open(metadata_path, 'r') as f:
                info_dict = yaml.safe_load(f)['rosbag2_bagfile_information']
                # Dauer in Nanosekunden konvertieren zu Sekunden
                self.bag_duration = float(info_dict['duration']['nanoseconds']) * 1e-9
        else:
            self.bag_duration = 1.0 # Fallback
            
        self.progress = ProgressBar(0, self.bag_duration, term_width=80, fd=StringIO())

        # Typen-Mapping für dynamische Publisher erstellen
        self.topic_types = {}
        self.publishers_dict = {}
        for topic_type in self.reader.get_all_topics_and_types():
            self.topic_types[topic_type.name] = topic_type.type
            # Dynamischer Import der Message-Klasse
            msg_class = self.get_message_class(topic_type.type)
            if msg_class:
                self.publishers_dict[topic_type.name] = self.create_publisher(msg_class, topic_type.name, 256)
        
        self.clock_pub = self.create_publisher(Clock, '/clock', 256)

        self.init_time = None
        self.start = start
        self.duration = duration
        self.fail_count = 0
        self.time_subs = {}
        self.target_times = {}
        self.latest_stamps = {}
        self.start_stamp = None

        self.play()

    def get_message_class(self, type_str):
        """ Hilfsfunktion um ROS 2 Nachrichtentypen dynamisch anhand des Strings zu importieren """
        try:
            parts = type_str.split('/')
            if len(parts) == 3: # z.B. std_msgs/msg/Header
                package, _, cls = parts
            else: # z.B. std_msgs/Header
                package, cls = parts
            module = __import__(f'{package}.msg', fromlist=[cls])
            return getattr(module, cls)
        except Exception as e:
            self.get_logger().error(f"Failed to import message class for {type_str}: {e}")
            return None

    def update_time_subs(self):
        # ROS 2 Äquivalent zu rospy.get_published_topics()
        for topic_name, topic_types in self.get_topic_names_and_types():
            if 'read_until' in topic_name and any('std_msgs/msg/Header' in t or 'std_msgs/Header' in t for t in topic_types):
                if topic_name not in self.time_subs:
                    print('ADD SUBSCRIBER:', topic_name)
                    # In ROS 2 wird der Callback-Argument-Pass über Lambda gelöst
                    self.time_subs[topic_name] = self.create_subscription(
                        Header, 
                        topic_name, 
                        lambda msg, tn=topic_name: self.time_callback(msg, tn), 
                        10
                    )

    def time_callback(self, header_msg, topic_name):
        if header_msg.frame_id not in self.target_times:
            self.target_times[header_msg.frame_id] = {}

        # Konvertiere builtin_interfaces/msg/Time zu rclpy.time.Time für Vergleiche
        stamp = Time.from_msg(header_msg.stamp)
        if topic_name not in self.target_times[header_msg.frame_id] or self.target_times[header_msg.frame_id][topic_name] < stamp:
            self.target_times[header_msg.frame_id][topic_name] = stamp

    def read_next_message(self):
        """ Liest und deserialisiert die nächste Nachricht aus dem ROS 2 Bag """
        if self.reader.has_next():
            topic, data, timestamp_ns = self.reader.read_next()
            msg_class = self.get_message_class(self.topic_types[topic])
            msg = deserialize_message(data, msg_class)
            # ROS 2 Bags nutzen interne Zeitstempel in Nanosekunden (Int)
            stamp = Time(nanoseconds=timestamp_ns)
            return topic, msg, stamp
        raise StopIteration

    def play_realtime(self, duration_limit):
        try:
            topic, msg, stamp = self.read_next_message()
        except StopIteration:
            return None, None, None

        stamp_wall = time.time()
        start_stamp = stamp
        start_stamp_wall = stamp_wall
        self.start_stamp = start_stamp

        while rclpy.ok() and (stamp - start_stamp) < duration_limit:
            stamp_wall = time.time()
            elapsed_stamp = stamp - start_stamp
            
            # ROS 2 nutzt Sekunden als Float oder Nanosekunden direkt
            if (stamp_wall - start_stamp_wall) < (elapsed_stamp.nanoseconds * 1e-9):
                time.sleep(1e-6)
                self.update_time_subs()
                continue

            clock_msg = Clock()
            clock_msg.clock = stamp.to_msg()

            if self.init_time is None:
                self.init_time = stamp

            # Startzeit-Offset prüfen
            if self.start and (stamp - self.init_time) < Duration(seconds=float(self.start)):
                start_stamp = stamp
            else:
                self.clock_pub.publish(clock_msg)
                if topic in self.publishers_dict:
                    self.publishers_dict[topic].publish(msg)

            try:
                topic, msg, stamp = self.read_next_message()
            except StopIteration:
                break

        return topic, msg, stamp

    def print_progress(self, stamp):
        self.stdscr.clear()
        self.stdscr.addstr(0, 0, 'topic')
        self.stdscr.addstr(0, 25, 'time')

        line = 1
        for target in self.target_times:
            if target not in self.publishers_dict:
                continue

            for sub_name in self.target_times[target]:
                target_time = self.target_times[target][sub_name]
                self.stdscr.addstr(line, 0, sub_name[:-11])
                
                # ROS 2: .sec und .nanosec (Singular!)
                t_msg = target_time.to_msg()
                self.stdscr.addstr(line, 25, '%.6f' % (t_msg.sec + t_msg.nanosec * 1e-9))

                latest_stamp = Time.from_msg(self.latest_stamps[target].stamp)
                residual = target_time - latest_stamp
                residual_sec = residual.nanoseconds * 1e-9

                color = 1 if residual_sec > 0.0 else 2
                self.stdscr.addstr(line, 50, '%.5f' % residual_sec, curses.color_pair(color))
                line += 1

        if not hasattr(self, 'prev_stamp'):
            self.prev_stamp = stamp
            self.prev_stamp_wall = time.time()
            self.processing_speed = 1.0
        elif time.time() - self.prev_stamp_wall > 1.0:
            sim_duration = (stamp - self.prev_stamp).nanoseconds * 1e-9
            wall_duration = time.time() - self.prev_stamp_wall
            self.processing_speed = sim_duration / wall_duration
            self.prev_stamp = stamp
            self.prev_stamp_wall = time.time()

        stamp_msg = stamp.to_msg()
        self.stdscr.addstr(line, 0, 'current_stamp')
        self.stdscr.addstr(line, 25, '%.6f' % (stamp_msg.sec + stamp_msg.nanosec * 1e-9))
        self.stdscr.addstr(line, 50, '(x%.2f)' % self.processing_speed)

        elapsed = (stamp - self.start_stamp).nanoseconds * 1e-9
        self.progress.fd = StringIO()
        try:
            self.progress.update(elapsed)
        except:
            pass
        self.stdscr.addstr(line + 1, 0, '----------')
        self.stdscr.addstr(line + 2, 0, self.progress.fd.getvalue())

        self.stdscr.refresh()

    def check_stamp(self, topic, msg):
        if topic not in self.target_times:
            return True

        if self.fail_count > 10:
            self.fail_count = 0
            return True

        target_time_map = self.target_times[topic]
        for sub_name in target_time_map:
            self.latest_stamps[topic] = msg.header
            msg_stamp = Time.from_msg(msg.header.stamp)
            if msg_stamp > target_time_map[sub_name]:
                self.fail_count += 1
                return False

        self.fail_count = 0
        return True

    def play(self):
        print('playing realtime for 5.0[sec]')
        topic, msg, stamp = self.play_realtime(Duration(seconds=5.0))
        if topic is None:
            print("Bag empty or early end.")
            return
            
        self.update_time_subs()

        print('playing as fast as possible')
        self.stdscr = curses.initscr()
        curses.start_color()
        curses.noecho()

        curses.init_pair(1, curses.COLOR_BLUE, curses.COLOR_WHITE)
        curses.init_pair(2, curses.COLOR_RED, curses.COLOR_WHITE)

        try:
            while rclpy.ok():
                if not self.check_stamp(topic, msg):
                    self.update_time_subs()
                    self.print_progress(stamp)
                    time.sleep(0.1)
                    continue

                clock_msg = Clock()
                clock_msg.clock = stamp.to_msg()

                if self.duration:
                    if (stamp - self.init_time) > Duration(seconds=float(self.duration)):
                        break

                self.clock_pub.publish(clock_msg)
                if topic in self.publishers_dict:
                    self.publishers_dict[topic].publish(msg)
                
                try:
                    topic, msg, stamp = self.read_next_message()
                except StopIteration:
                    break
        except Exception as e:
            # Im Fehlerfall Curses sauber beenden, bevor wir printen!
            curses.echo()
            curses.endwin()
            print(f"Exception encountered: {e}")
            clock_msg = Clock()
            clock_msg.clock = (stamp + Duration(seconds=30.0)).to_msg()
            self.clock_pub.publish(clock_msg)
            time.sleep(0.5)
            return

        curses.echo()
        curses.endwin()


def main():
    # In ROS 2 werten wir sys.argv direkt aus, da rclpy Argumente filtern kann
    rclpy.init(args=sys.argv)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('input_bag', help='Bag folder (or file) to be played')
    parser.add_argument('-s', '--start', help='start sec seconds into the bag')
    parser.add_argument('-u', '--duration', help='play only sec seconds into the bag')
    
    # Entferne ROS-interne Parameter falls vorhanden vor dem Parsen
    clean_args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    args = parser.parse_args(clean_args)

    try:
        node = BagPlayer(args.input_bag, args.start, args.duration)
        # Da der Player die Schleife manuell kontrolliert (play()), 
        # brauchen wir hier kein dauerhaftes rclpy.spin(node).
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
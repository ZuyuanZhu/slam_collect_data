#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image, Imu, NavSatFix
from nav_msgs.msg import Odometry, Path
import rosbag

class DataCollector:
    def __init__(self):
        # ROS node initialization
        rospy.init_node('data_collector', anonymous=True)
        self.bag = rosbag.Bag('jackal_zed.bag', 'w')

        # Counter for sequence numbers
        self.seq = 0

        # Subscriptions
        sub_img = message_filters.Subscriber("/zed/zed_node/left/image_rect_color", Image)
        sub_imu = message_filters.Subscriber("/zed/imu/data", Imu)
        sub_odom = message_filters.Subscriber("/jackal_velocity_controller/odom", Odometry)
        sub_odom_filtered = message_filters.Subscriber("/odometry/filtered", Odometry)
        sub_odom_zed = message_filters.Subscriber("/zed/zed_node/odom", Odometry)
        sub_odom_path_zed = message_filters.Subscriber("/zed/zed_node/path_odom", Path)
        sub_gps = message_filters.Subscriber("/navsat/fix", NavSatFix)

        # ApproximateTimeSynchronizer to synchronize Image, Imu, Odometry and GPS data
        # start with a lower slop value (e.g., 0.1 or even 0.05 seconds) and see if that works
        # for the application, and then adjust as necessary based on the observed results.
        ts = message_filters.ApproximateTimeSynchronizer([sub_img, sub_imu, sub_odom, sub_odom_filtered, sub_odom_zed, sub_odom_path_zed, sub_gps], queue_size=10, slop=0.5)
        ts.registerCallback(self.callback)

    def callback(self, img, imu, odom, odom_filtered, odom_zed, odom_path_zed, gps):
        # Update sequence number for each message's frame
        frame_id = str(self.seq).zfill(10)
        img.header.frame_id = frame_id
        imu.header.frame_id = frame_id
        odom.header.frame_id = frame_id
        odom_filtered.header.frame_id = frame_id
        odom_zed.header.frame_id = frame_id
        odom_path_zed.header.frame_id = frame_id
        gps.header.frame_id = frame_id

        # Increase the sequence number
        self.seq += 1

        self.bag.write('/zed/zed_node/left/image_rect_color', img)
        self.bag.write('/zed/imu/data', imu)
        self.bag.write('/jackal_velocity_controller/odom', odom)
        self.bag.write('/jackal_velocity_controller/odom', odom_filtered)
        self.bag.write('/odometry/filtered', odom_zed)
        self.bag.write('/zed/zed_node/path_odom', odom_path_zed)
        self.bag.write('/navsat/fix', gps)

if __name__ == '__main__':
    dc = DataCollector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        dc.bag.close()

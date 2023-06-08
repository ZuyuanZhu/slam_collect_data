#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image, Imu, NavSatFix
from nav_msgs.msg import Odometry
import rosbag

class DataCollector:
    def __init__(self):
        # ROS node initialization
        rospy.init_node('data_collector', anonymous=True)
        self.bag = rosbag.Bag('jackal_zed.bag', 'w')

        # Subscriptions
        sub_img = message_filters.Subscriber("/zed/rgb/image_raw_color", Image)
        sub_imu = message_filters.Subscriber("/zed/imu/data", Imu)
        sub_odom = message_filters.Subscriber("/jackal_velocity_controller/odom", Odometry)
        sub_gps = message_filters.Subscriber("/jackal_gps/fix", NavSatFix)

        # ApproximateTimeSynchronizer to synchronize Image, Imu, Odometry and GPS data
        ts = message_filters.ApproximateTimeSynchronizer([sub_img, sub_imu, sub_odom, sub_gps], queue_size=10, slop=0.5)
        ts.registerCallback(self.callback)

    def callback(self, img, imu, odom, gps):
        self.bag.write('/zed/rgb/image_raw_color', img)
        self.bag.write('/zed/imu/data', imu)
        self.bag.write('/jackal_velocity_controller/odom', odom)
        self.bag.write('/jackal_gps/fix', gps)

if __name__ == '__main__':
    dc = DataCollector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        dc.bag.close()


#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Int16
from ackermann_msgs.msg import AckermannDrive
import time

def test_rc_car_control():
    rospy.init_node('test_rc_car_control')

    # Publishers
    auto_mode_pub = rospy.Publisher('/auto_mode', Bool, queue_size=10)
    steer_pub = rospy.Publisher('/auto_cmd/steer', AckermannDrive, queue_size=10)
    throttle_pub = rospy.Publisher('/auto_cmd/throttle', Int16, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        # Test Auto Mode
        auto_mode = Bool()
        auto_mode.data = True
        auto_mode_pub.publish(auto_mode)
        rospy.loginfo("Published Auto Mode: True")

        # Test Steering
        steer_cmd = AckermannDrive()
        steer_cmd.steering_angle = 15  # Test angle, adjust as needed
        steer_cmd.steering_angle_velocity = 5  # Test velocity, adjust as needed
        steer_pub.publish(steer_cmd)
        rospy.loginfo("Published Steering Command: Angle = 15, Velocity = 5")

        # Test Throttle
        throttle_cmd = Int16()
        throttle_cmd.data = 50  # Test speed percentage, adjust as needed
        throttle_pub.publish(throttle_cmd)
        rospy.loginfo("Published Throttle Command: 50%")

        rate.sleep()
        time.sleep(5)  # Adjust duration for each test cycle as needed

if __name__ == '__main__':
    try:
        test_rc_car_control()
    except rospy.ROSInterruptException:
        pass

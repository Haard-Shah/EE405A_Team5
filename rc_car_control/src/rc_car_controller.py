#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Int16
from ackermann_msgs.msg import AckermannDrive

class RCCarController:
    def __init__(self, auto_mode_topic='/auto_mode', steer_topic='/auto_cmd/steer', throttle_topic='/auto_cmd/throttle'):
        # Initialize node
        rospy.init_node('rc_car_controller')

        # PWM bounds and steering constants
        self.pwm_upper_bound = 1900  # Full right
        self.pwm_lower_bound = 1100  # Full left
        self.pwm_neutral = 1500      # Neutral
        self.max_steering_angle = 30 # Max steering angle in degrees
        self.max_speed = 5           # adjust this later

        # Subscribers
        self.auto_mode_sub = rospy.Subscriber(auto_mode_topic, Bool, self.auto_mode_callback)
        self.steer_sub = rospy.Subscriber(steer_topic, AckermannDrive, self.steer_callback)
        self.throttle_sub = rospy.Subscriber(throttle_topic, Int16, self.throttle_callback)

        # Publishers
        self.steer_pwm_pub = rospy.Publisher('/rc_car/steer_pwm', Int16, queue_size=10)
        self.throttle_pwm_pub = rospy.Publisher('/rc_car/throttle_pwm', Int16, queue_size=10)

        # Current state
        self.auto_mode = False
        self.current_pwm = self.pwm_neutral
        self.target_pwm = self.pwm_neutral
        self.speed = 0

    def auto_mode_callback(self, msg):
        self.auto_mode = msg.data
        rospy.loginfo("Autonomous mode: %s" % ("ON" if self.auto_mode else "OFF"))

    def steer_callback(self, ackermann_cmd):
        if self.auto_mode:
            # Map steering_angle to target PWM
            self.target_pwm = self.steering_angle_to_pwm(ackermann_cmd.steering_angle)

            if ackermann_cmd.steering_angle_velocity > 0:
                # Calculate the number of steps for transition based on steering_angle_velocity
                transition_steps = int(abs(self.target_pwm - self.current_pwm) / ackermann_cmd.steering_angle_velocity)
                for _ in range(transition_steps):
                    # Gradually move towards target PWM
                    self.current_pwm += (self.target_pwm - self.current_pwm) / transition_steps
                    self.publish_steering_command(int(self.current_pwm))
                    rospy.sleep(0.1)  # Adjust sleep duration as needed for smoother transition
            else:
                # If steering_angle_velocity is not defined, set directly
                self.current_pwm = self.target_pwm
                self.publish_steering_command(self.current_pwm)

    def throttle_callback(self, msg):
        if self.auto_mode:
            self.speed = msg.data
            rospy.loginfo("Speed command: %d" % self.speed)
            pwm_value = self.speed_to_pwm(self.speed)
            self.publish_throttle_command(pwm_value)

    def publish_steering_command(self, pwm_value):
        pwm_signal = Int16()
        pwm_signal.data = pwm_value
        self.steer_pwm_pub.publish(pwm_signal)

    def publish_throttle_command(self, pwm_value):
        pwm_signal = Int16()
        pwm_signal.data = pwm_value
        self.throttle_pwm_pub.publish(pwm_signal)

    def steering_angle_to_pwm(self, angle):
        rospy.loginfo("Angle command: %s rads" % angle)
        #TODO: May need to add rads to deg or rads to pwm logic here.
        # Map steering angle to PWM value
        return int(self.pwm_neutral + (angle / self.max_steering_angle) * (self.pwm_upper_bound - self.pwm_neutral))

    def speed_to_pwm(self, speed):
        # Convert speed to PWM value
        if speed >= 0:
            return int(self.pwm_neutral + (speed / float(self.max_speed)) * (self.pwm_upper_bound - self.pwm_neutral))
        else:
            return int(self.pwm_neutral + (speed / float(self.max_speed)) * (self.pwm_neutral - self.pwm_lower_bound))

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Publish current steering command continuously
            self.publish_steering_command(self.current_pwm)
            rate.sleep()

if __name__ == '__main__':
    auto_mode_topic = rospy.get_param('~auto_mode_topic', '/auto_mode')
    steer_topic = rospy.get_param('~steer_topic', '/auto_cmd/steer')
    throttle_topic = rospy.get_param('~throttle_topic', '/auto_cmd/throttle')

    controller = RCCarController(auto_mode_topic, steer_topic, throttle_topic)
    controller.run()

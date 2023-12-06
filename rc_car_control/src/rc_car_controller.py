#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Int16
from ackermann_msgs.msg import AckermannDrive


class RCCarController:
    def __init__(self, auto_mode_topic='/auto_mode', drive_command_topic='/car_1/command'):
        # Initialize node
        rospy.init_node('rc_car_controller')

        # PWM bounds and steering constants
        self.steer_pwm_upper_bound = 1900  # Full right
        self.steer_pwm_lower_bound = 1100  # Full left
        self.pwm_neutral = 1500      # Neutral

        self.speed_F_pwm_upper_bound = 1460  # slowest forward
        self.speed_F_pwm_lower_bound = 1474  # fastest forward
        self.speed_R_pwm_upper_bound = 1590  # fastest reverse 
        self.speed_R_pwm_lower_bound = 1575  # slowest reverse

        self.prev_speed = 0


        self.max_steering_angle = 15 # Max steering angle in degrees
        self.max_speed = 0.6           # adjust this later

        # Subscribers
        self.auto_mode_sub = rospy.Subscriber(auto_mode_topic, Bool, self.auto_mode_callback)
        self.drive_command_sub = rospy.Subscriber(drive_command_topic, AckermannDrive, self.drive_command_callback)
        # self.steer_sub = rospy.Subscriber(steer_topic, AckermannDrive, self.steer_callback)
        # self.throttle_sub = rospy.Subscriber(throttle_topic, Int16, self.throttle_callback)

        # Publishers
        self.steer_pwm_pub = rospy.Publisher('/auto_cmd/steer', Int16, queue_size=10)
        self.throttle_pwm_pub = rospy.Publisher('/auto_cmd/throttle', Int16, queue_size=10)
        self._autonomous_pub = rospy.Publisher('/auto_mode', Bool, queue_size=10)

        # Current state
        self.auto_mode = False
        self.current_pwm = self.pwm_neutral
        self.target_pwm = self.pwm_neutral
        self.speed = 0

    def auto_mode_callback(self, msg):
        self.auto_mode = msg.data
        rospy.loginfo("Autonomous mode: %s" % ("ON" if self.auto_mode else "OFF"))

    def drive_command_callback(self, ackermann_cmd):
        # drive_command = AckermannDrive()
        # drive_command.data = ackermann_cmd.data

        # steer_cmd = steer_cmd(ackermann_cmd.steering_angle, ackermann_cmd.steering_angle_velocity)

        self.throttle_callback(ackermann_cmd)
        self.steer_callback(ackermann_cmd)

    def steer_callback(self, steer_cmd):
        if self.auto_mode:
            # Map steering_angle to target PWM
            steer_deg = steer_cmd.steering_angle * 57.29577951 # rad to deg
            self.target_pwm = self.steering_angle_to_pwm(steer_deg)

            # rospy.loginfo("Angle command: %s deg   PWM: %s" % (steer_deg, self.target_pwm))

            if steer_cmd.steering_angle_velocity > 0:
                # Calculate the number of steps for transition based on steering_angle_velocity
                transition_steps = int(abs(self.target_pwm - self.current_pwm) / steer_cmd.steering_angle_velocity)
                for _ in range(transition_steps):
                    # Gradually move towards target PWM
                    self.current_pwm += (self.target_pwm - self.current_pwm) / transition_steps
                    self.publish_steering_command(int(self.current_pwm))
                    rospy.sleep(0.1)  # Adjust sleep duration as needed for smoother transition
            else:
                # If steering_angle_velocity is not defined, set directly
                self.current_pwm = self.target_pwm
                self.publish_steering_command(self.current_pwm)

    def throttle_callback(self, ackermann_cmd):
        if self.auto_mode:
            self.speed = ackermann_cmd.speed
            pwm_value = self.speed_to_pwm(self.speed)
            rospy.loginfo("Speed command: %f %d" % (self.speed, pwm_value))
            self.publish_throttle_command(pwm_value)
            
            self.prev_speed = self.speed

    def publish_steering_command(self, pwm_value):
        pwm_signal = Int16()
        pwm_signal.data = pwm_value
        self.steer_pwm_pub.publish(pwm_signal)

    def publish_throttle_command(self, pwm_value):

        if (pwm_value > 1500) and (self.prev_speed > 0):
            pwm_signal = Int16()
            pwm_signal.data = 1500
            self.throttle_pwm_pub.publish(pwm_signal)
            print("sent 1500")

            rospy.sleep(1)


        pwm_signal = Int16()
        pwm_signal.data = pwm_value
        self.throttle_pwm_pub.publish(pwm_signal)

    def steering_angle_to_pwm(self, angle):
        #TODO: May need to add rads to deg or rads to pwm logic here.
        # Map steering angle to PWM value
        return int(self.pwm_neutral - (angle / self.max_steering_angle) * (self.steer_pwm_upper_bound - self.pwm_neutral))

    def speed_to_pwm(self, speed):
        # Convert speed to PWM value
        if speed >= 0:
            # return int(self.pwm_neutral + max((speed / float(self.max_speed)), 0.1) * (self.speed_pwm_upper_bound - self.pwm_neutral))
            return (self.speed_F_pwm_lower_bound - ((speed/self.max_speed) * (self.speed_F_pwm_lower_bound - self.speed_F_pwm_upper_bound)))
        else:
            # return int(self.pwm_neutral + (speed / float(self.max_speed)) * (self.pwm_neutral - self.speed_pwm_lower_bound))
            return (self.speed_R_pwm_lower_bound + ((-speed/self.max_speed) * (self.speed_R_pwm_upper_bound - self.speed_R_pwm_lower_bound)))

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        # Enable autonomous mode
        autonomous_on = Bool()
        autonomous_on.data = 1
        self._autonomous_pub.publish(autonomous_on)

        try:
            while not rospy.is_shutdown():
                # Publish current steering command continuously
                self.publish_steering_command(self.current_pwm)
                rate.sleep()

        except KeyboardInterrupt:
            print("Keyboard interupt: Shutting down")
            autonomous_on.data = False
            self._autonomous_pub.publish(autonomous_on)
        

if __name__ == '__main__':
    auto_mode_topic = rospy.get_param('~auto_mode_topic', '/auto_mode')
    steer_topic = rospy.get_param('~steer_topic', '/auto_cmd/steer')
    throttle_topic = rospy.get_param('~throttle_topic', '/auto_cmd/throttle')
    drive_command_topic = rospy.get_param('~drive_command_topic', '/car_1/command')

    # controller = RCCarController(auto_mode_topic, steer_topic, throttle_topic)
    controller = RCCarController(auto_mode_topic, drive_command_topic)
    controller.run()


# add a ctrl + C interrupt to stop
# auto start in auto mode at start

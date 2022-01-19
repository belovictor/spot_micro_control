#!/usr/bin/python

import rospy
import socket
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from math import pi
from spotcontrolreceiver import SpotControlReceiver


def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    return IP


class SpotMicroRemoteControl():
    DEFAULT_UDP_PORT = 12345

    MODE_IDLE = 0
    MODE_STAND = 1
    MODE_ANGLE = 2
    MODE_WALK = 3

    MAX_ROLL_DEG = 45
    MAX_YAW_DEG = 45
    MAX_PATCH_DEG = 45

    MAX_FORWARD_SPEED = 0.1
    MAX_STRAFE_SPEED = 0.1
    MAX_YAW_SPEED_DEG = 15

    def __init__(self):

        self._angle_cmd_msg = Vector3()
        self._angle_cmd_msg.x = 0
        self._angle_cmd_msg.y = 0
        self._angle_cmd_msg.z = 0

        self._vel_cmd_msg = Twist()
        self._vel_cmd_msg.linear.x = 0
        self._vel_cmd_msg.linear.y = 0
        self._vel_cmd_msg.linear.z = 0
        self._vel_cmd_msg.angular.x = 0
        self._vel_cmd_msg.angular.y = 0
        self._vel_cmd_msg.angular.z = 0

        self._walk_event_cmd_msg = Bool()
        self._walk_event_cmd_msg.data = True  # Mostly acts as an event driven action on receipt of a true message

        self._stand_event_cmd_msg = Bool()
        self._stand_event_cmd_msg.data = True

        self._idle_event_cmd_msg = Bool()
        self._idle_event_cmd_msg.data = True

        rospy.loginfo("Setting Up the Spot Micro Remote Control Node...")

        # Set up and title the ros node for this code
        rospy.init_node('spot_micro_remote_control')

        # Create publishers for commanding velocity, angle, and robot states
        self._ros_pub_angle_cmd = rospy.Publisher('/angle_cmd', Vector3, queue_size=1)
        self._ros_pub_vel_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._ros_pub_walk_cmd = rospy.Publisher('/walk_cmd', Bool, queue_size=1)
        self._ros_pub_stand_cmd = rospy.Publisher('/stand_cmd', Bool, queue_size=1)
        self._ros_pub_idle_cmd = rospy.Publisher('/idle_cmd', Bool, queue_size=1)

        self._receiver = None

        rospy.loginfo("Remote control node publishers corrrectly initialized")

    def reset_all_motion_commands_to_zero(self):
        '''Reset body motion cmd states to zero and publish zero value body motion commands'''

        self._vel_cmd_msg.linear.x = 0
        self._vel_cmd_msg.linear.y = 0
        self._vel_cmd_msg.linear.z = 0
        self._vel_cmd_msg.angular.x = 0
        self._vel_cmd_msg.angular.y = 0
        self._vel_cmd_msg.angular.z = 0

        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

    def reset_all_angle_commands_to_zero(self):
        '''Reset angle cmd states to zero and publish them'''

        self._angle_cmd_msg.x = 0
        self._angle_cmd_msg.y = 0
        self._angle_cmd_msg.z = 0

        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

    def run(self):
        print('Run your SpotControl application and connect to ' + get_ip() + ':' + str(self.DEFAULT_UDP_PORT))
        self._receiver = SpotControlReceiver()
        # Publish all body motion commands to 0
        self.reset_all_motion_commands_to_zero()
        while not rospy.is_shutdown():
            data_string = self._receiver.receive()
            print(data_string)
            command_array = data_string.split(' ')
            if len(command_array) < 2:
                print('Command has less then 2 components, ignoring')
                continue
            if command_array[0] == 'LEFT' or command_array[0] == 'RIGHT':
                print('Joystick command received')
                if len(command_array) < 3:
                    print('Joystick commands should have 2 arguments at least')
                    continue
                axis_x = float(command_array[1])
                axis_y = float(command_array[2])
                if command_array[0] == 'LEFT':
                    if self.mode == self.MODE_WALK:
                        self._vel_cmd_msg.linear.x = axis_y * self.MAX_FORWARD_SPEED * -1
                        self._vel_cmd_msg.linear.y = axis_x * self.MAX_STRAFE_SPEED
                        print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s ' \
                              % (self._vel_cmd_msg.linear.x, self._vel_cmd_msg.linear.y,
                                 self._vel_cmd_msg.angular.z * 180 / pi))
                        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                    elif self.mode == self.MODE_ANGLE:
                        self._angle_cmd_msg.x = pi / 180 * axis_x * self.MAX_ROLL_DEG
                        self._angle_cmd_msg.y = pi / 180 * axis_y * self.MAX_PATCH_DEG
                        print('Cmd Values: phi: %1.3f deg, theta: %1.3f deg, psi: %1.3f deg ' \
                              % (
                                  self._angle_cmd_msg.x * 180 / pi, self._angle_cmd_msg.y * 180 / pi,
                                  self._angle_cmd_msg.z * 180 / pi))
                        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
                if command_array[0] == 'RIGHT':
                    if self.mode == self.MODE_WALK:
                        self._vel_cmd_msg.angular.z = pi / 180 * axis_x * self.MAX_YAW_SPEED_DEG
                        self._vel_cmd_msg.linear.x = axis_y * self.MAX_FORWARD_SPEED * -1
                        print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s ' \
                              % (self._vel_cmd_msg.linear.x, self._vel_cmd_msg.linear.y,
                                 self._vel_cmd_msg.angular.z * 180 / pi))
                        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                    elif self.mode == self.MODE_ANGLE:
                        self._angle_cmd_msg.z = pi / 180 * axis_x * self.MAX_YAW_DEG * -1
                        print('Cmd Values: phi: %1.3f deg, theta: %1.3f deg, psi: %1.3f deg ' \
                              % (
                                  self._angle_cmd_msg.x * 180 / pi, self._angle_cmd_msg.y * 180 / pi,
                                  self._angle_cmd_msg.z * 180 / pi))
                        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
            elif command_array[0] == 'ACTIVE':
                print('Active command received')
                if command_array[1] == '1':
                    rospy.loginfo('Stand command issued from remote control.')
                    self._ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                    self.mode = self.MODE_STAND
                    rospy.loginfo('Entering angle command mode.')
                    self.reset_all_angle_commands_to_zero()
                    self.mode = self.MODE_ANGLE
                else:
                    rospy.loginfo('Idle command issued from remote control.')
                    self._ros_pub_idle_cmd.publish(self._idle_event_cmd_msg)
                    self.mode = self.MODE_IDLE
            elif command_array[0] == 'WALK':
                print('Walk command received')
                if command_array[1] == '1':
                    rospy.loginfo('Entering walk command mode.')
                    self.reset_all_angle_commands_to_zero()
                    self._ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)
                    self.mode = self.MODE_WALK
                else:
                    rospy.loginfo('Entering angle command mode.')
                    self.reset_all_angle_commands_to_zero()
                    self._ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                    self.mode = self.MODE_ANGLE
            else:
                print('Unsupported command received')

if __name__ == "__main__":
    smc = SpotMicroRemoteControl()
    smc.run()

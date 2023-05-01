# generate python script to control velocity of PX4 based rover using SET_POSITION_TARGET_LOCAL_NED in OFFBOARD mode

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, OverrideRCIn, ManualControl
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import PositionTarget
from sensor_msgs.msg import NavSatFix


class Controller:
    def __init__(self):
        rospy.init_node("controller", anonymous=True)
        self.rate = rospy.Rate(30)
        self.current_state = State()
        self.local_pos_pub = rospy.Publisher(
            "/rover/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.local_vel_pub = rospy.Publisher(
            "/rover/mavros/setpoint_raw/local", PositionTarget, queue_size=10
        )
        self.rc_pub = rospy.Publisher(
            "/rover/mavros/rc/override", OverrideRCIn, queue_size=10
        )
        self.manual_pub = rospy.Publisher(
            "/rover/mavros/manual_control/send", ManualControl, queue_size=10
        )
        self.arming_client = rospy.ServiceProxy("/rover/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/rover/mavros/set_mode", SetMode)
        self.pose = PoseStamped()
        self.vel = PositionTarget()
        self.vel.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.vel.type_mask = (
            PositionTarget.IGNORE_PX
            + PositionTarget.IGNORE_PY
            + PositionTarget.IGNORE_PZ
            + PositionTarget.IGNORE_AFX
            + PositionTarget.IGNORE_AFY
            + PositionTarget.IGNORE_AFZ
            + PositionTarget.IGNORE_YAW_RATE
        )
        self.vel.velocity.x = 0
        self.vel.velocity.y = 0
        self.vel.velocity.z = 0
        self.vel.yaw = 0
        self.vel.yaw_rate = 0
        self.current_state.connected = False
        self.current_state.armed = False
        self.current_state.mode = "OFFBOARD"

    def connect(self):
        print(self.current_state)
        rospy.loginfo("Waiting for FCU connection...")
        while not self.current_state.connected:
            self.rate.sleep()
        rospy.loginfo("Connected")

    def arm(self):
        rospy.loginfo("Arming...")
        while not self.current_state.armed:
            self.arming_client(True)
            self.rate.sleep()
        rospy.loginfo("Armed")

    def state_cb(self, msg):
        self.current_state.connected = msg.connected
        self.current_state.armed = msg.armed
        self.current_state.mode = msg.mode
        print(msg.mode)

    def set_vel(self, vx, vy, vz, yaw):
        self.vel.velocity.x = vx
        self.vel.velocity.y = vy
        self.vel.velocity.z = vz
        self.vel.yaw = yaw
        self.local_vel_pub.publish(self.vel)

    def send_manual_control(self, x, y, z, r):
        msg = ManualControl()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.r = r
        self.manual_pub.publish(msg)

    def RC_channel_override(self, channel, value):
        msg = OverrideRCIn()
        # make all channels 2000

        for i in range(0, 8):
            msg.channels[i] = 2000

        self.rc_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    # set rate
    rate = rospy.Rate(100)
    controller = Controller()
    rospy.Subscriber("/rover/mavros/state", State, controller.state_cb)

    print("Waiting for FCU connection...")

    controller.connect()

    controller.send_manual_control(0, 0, 0, 0)

    print("Arming...")
    controller.arm()

    controller.set_mode_client(0, "STABILIZED")

    while not rospy.is_shutdown():
        controller.send_manual_control(0, 50, 600, 0)

        if controller.current_state.mode != "STABILIZED":
            controller.set_mode_client(0, "STABILIZED")

        controller.rate.sleep()

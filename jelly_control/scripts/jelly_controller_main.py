#!/usr/bin/env python
import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import jelly_locomotion.jelly_gaits as gaits

class JellyGUI:
    def __init__(self):
        # TODO check with jelly_web that thses topcis are correct
        rospy.Subscriber("/jelly_gui/command", String, self.update_status)
        self.pub = rospy.Publisher("/jelly_gui/status", String, queue_size=1)
        self.mode = 0
        self.cmd = 0

        # initalize
    def update_status(self, msg):
        return msg.data

    def read_command(self):
        return self.mode, self.cmd

    def write(self, msg):
        # write to console
        rospy.logerr(msg)
        s = String()
        s.data = msg
        self.pub.publish(s)


class JellyRobot:
    def __init__(self):
        # set control rate
        self.rate = rospy.Rate(200)

        # collect parameters of robot
        self.joint_names = rospy.get_param("/jelly_hardware/joint_names")
        self.joint_to_idx = {}
        for idx, joint in enumerate(self.joint_names):
            self.joint_to_idx[joint] = idx

        self.base_link   = rospy.get_param("/jelly_hardware/base_link")
        self.fl_link     = rospy.get_param("/jelly_hardware/fl_link")
        self.fr_link     = rospy.get_param("/jelly_hardware/fr_link")
        self.rl_link     = rospy.get_param("/jelly_hardware/rl_link")
        self.rr_link     = rospy.get_param("/jelly_hardware/rr_link")

        self.gear_ratio  = rospy.get_param("/jelly_hardware/gear_ratio")
        self.joint_directions = rospy.get_param("/jelly_hardware/joint_directions")

        # set up publishers for odrive
        self.odrives = rospy.get_param("/jelly_hardware/odrive_ids")
        self.motor_publishers = []

        for odrive in self.odrives:
            id = odrive["id"]
            a1 = odrive["axis1"]
            a2 = odrive["axis2"]
            # TODO change to correct message type and topic
            pi = rospy.Publisher("/jelly_hardware/odrives" + str(id)  +"/command", Float64MultiArray, queue_size=1)
            self.motor_publishers.append(pi) # set up odrive
            rospy.logerr(a1)
            rospy.logerr(a2)

            pass

        # TODO change to correct message type and topic
        self.vesc_pub = rospy.Publisher("/jelly_hardware/vesc_cmd/command", Float64, queue_size=1)


        # initalize state
        self.joint_positions  = [0.0] * len(self.joint_names)
        # self.joint_velocities = [0.0] * len(self.joint_names)
        # self.joint_torques    = [0.0] * len(self.joint_names)

        # collect parameters of controller
        self._home_position = rospy.get_param("/jelly_control/home_position")
        self._rolling_position = rospy.get_param("/jelly_control/rolling_position")


        # Initalize Gaits
        self.total_gait_count = rospy.get_param("/jelly_control/total_gait_count")
        self.gait_index = 0
        ###################### Walking ###################################
        p1     = np.array(rospy.get_param("/jelly_control/walking_gait/p1"))
        p2     = np.array(rospy.get_param("/jelly_control/walking_gait/p2"))
        offset = np.array(rospy.get_param("/jelly_control/walking_gait/offset"))
        beta   = np.array(rospy.get_param("/jelly_control/walking_gait/beta"))
        p1 = p1 + offset;

        self.walking_gait = gaits.SimpleWalkingGait(beta, p1, p2, mode="reverse_crab")
        ######################################################################

        ####################### Turning ###################################
        p1     = np.array(rospy.get_param("/jelly_control/turning_gait/p1"))
        p2     = np.array(rospy.get_param("/jelly_control/turning_gait/p2"))
        flip = np.array([-1.0, -1.0, 1.0])

        p1l = p1
        p2l = p2
        p1r = p1 * flip
        p2r = p2 * flip

        self.turning_gait = gaits.TurningGait(p1l, p2l, p1r, p2r, mode="reverse_crab")
        ########################################################################

        ####################### Troting ###################################
        # offset = 0.07
        # p1 = np.array([offset + 1.85e-01, 4.5e-02, -3.04e-01])
        # p2 = np.array([-1.85e-01, 4.5e-02, -4.14e-01])
        # gait_controller = gaits.TrotGait(p1, p2, mode="reverse_crab")
        # T_cycle = 400
        ########################################################################

        ######################## SideStep ###################################
        # offset = 0.0
        # p1 = np.array([offset ,-0.1 +  0.5e-01, -4.24e-01])
        # p2 = np.array([offset, -0.1 + -0.5e-01, -4.24e-01])
        # gait_controller = gaits.SimpleSideGait(0.85, p1, p2, mode="reverse_crab")
        # gait_controller.set_height(0.20)
        # T_cycle = 2400
        ###########################################################################

        ######################### Bounding  ###################################
        # offset = 0.07
        # p1 = np.array([offset + 0.95e-01, -0.5e-01, -3.94e-01])
        # p2 = np.array([-1.25e-01        , -0.5e-01, -3.94e-01])
        # gait_controller = gaits.BoundGait(p1, p2, mode=None)
        # gait_controller.set_height(0.09)
        # T_cycle = 300
        #############################################################################

    def set_joints(self, cmds):
        for i, odrive in enumerate(self.odrives):
            a1 = odrive["axis1"]
            a2 = odrive["axis2"]
            idx1 = self.joint_to_idx[a1]
            idx2 = self.joint_to_idx[a2]
            # TODO change to correct message type and topic
            pub_i = self.motor_publishers[i] # set up odrive

            # TODO use correct custon message type
            msg = Message()
            msg.data1 = cmds[idx1]
            msg.data2 = cmds[idx2]
            pub_i.publish(msg)

        # TODO, assume cmded positions is the joint position
        self.joint_positions = cmds

    def move_joints(self, cmd, duration=1):
        points = 100
        curr_joints = np.array(self.joint_positions)
        cmd = np.array(cmd)
        for i in range(points):
            self.set_joints(curr_joints * (1.0 - float(i)/float(duration)) + cmd * float(i)/float(duration) )
            time.sleep(float(i/duration))
        self.set_joints(cmd)


    def update_joints(self, joint_msg):
        # TODO
        pass

    def calibrate(self):
        # TODO
        # insert calibration code
        pass


    def home(self, duration=1):
        self.move_joints(self._home_position, duration=duration)

    def command(self, mode, command):

        # increment gait
        self.gait_index = (self.gait_index + command*1)%self.total_gait_count
        gait_cmd = self.gait_index/self.total_gait_count

        # command motors appropriately
        if mode == self.mode:
            if self.mode == -1: #rolling  mode
                speed = 0.5 * command
                msg = Float64()
                msg.data = speed
                # write to vesc and joints
                self.vesc_pub.publish(msg)
                self.set_joints(self._rolling_position)

            elif self.mode == 0: # standing mode
                self.home(duration=0.1)

            elif self.mode == 1: # walking mode
                positions = self.walking_gait(gait_cmd)
                self.set_joints(positions, duration=switch_time)

            elif self.mode == 2: # Turning mode
                positions = self.turning_gait(gait_cmd)
                self.set_joints(positions, duration=switch_time)

        else:
            self.switch_to(mode)

    def switch_to(mode):
        switch_time = 2.5
        self.gait_index = 0
        gait_cmd = self.gait_index/self.total_gait_count

        if mode == -1:
            self.move_joints(self._rolling_position, duration=switch_time)

        elif self.mode == 0: # standing mode
            self.home(duration=switch_time)

        elif self.mode == 1: # walking mode
            positions = self.walking_gait(gait_cmd)
            self.move_joints(positions, duration=switch_time)

        elif self.mode == 2: # Turning mode
            positions = self.turning_gait(gait_cmd)
            self.move_joints(positions, duration=switch_time)

        else:
            self.home(duration=switch_time)

def parse_msg(msg):
    # -1 rolling
    # 0 standing
    # 1 walking front or back
    # 2 walking side to side

    mode = 0
    cmd  = 0

    # Rolling
    if msg == "roll_forward":
        mode = -1
        cmd = 1
    elif msg == "roll_stop":
        mode = -1
        cmd = 0
    elif msg == "roll_back":
        mode = -1
        cmd = -1

    # Standing
    elif msg == "stand_left":
        mode = 0
        cmd = -1
    elif msg == "stand_stop":
        mode = 0
        cmd = 0
    elif msg == "stand_right":
        mode = 0
        cmd = 1

    # Walking
    elif msg == "walk_forward":
        mode = 1
        cmd = 1
    elif msg == "walk_stop":
        mode = 1
        cmd = 0
    elif msg == "walk_back":
        mode = 1
        cmd = -1

    # Turning
    elif msg == "walk_left":
        mode = 2
        cmd = -1
    elif msg == "walk_right":
        mode = 2
        cmd = 1

    return mode, cmd

if __name__ == '__main__':
    # initialize the robot and GUi
    rospy.init_node('jelly_controller', anonymous=True)
    jelly = JellyRobot()
    gui = JellyGUI()
    gui.write("Jelly setup complete")

    # calibrate jelly
    gui.write("calibrating")
    jelly.calibrate()
    # jelly.home()
    gui.write("done calibrating")

    # control loop
    while not rospy.is_shutdown():

        # read from gui
        msg = gui.read_command()
        mode, cmd = parse_msg(msg)
        jelly.command(mode, cmd)
        jelly.rate.sleep()

#!/usr/bin/env python

"""
@author: David ter Kuile

Grasp controller to close with a determined error on position only
so to skip overheating.

"""

import rospy
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String, UInt8
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class GripperGrasp(object):
    def __init__(self):
        rospy.loginfo("Initializing Gripper Grasper...")

        # Get the params from param server
        self.last_state = None
        self.on_optimal_close = False
        self.on_optimal_open = False
        self.is_grasped_msg = Bool()

        self.controller_name = rospy.get_param("~controller_name", None)
        if not self.controller_name:
            rospy.logerr("No controller name found in param: ~controller_name")
            exit(1)
        self.real_joint_names = rospy.get_param("~real_joint_names", None)
        if not self.real_joint_names:
            rospy.logerr(
                "No real joint names given in param: ~real_joint_names")
            exit(1)
        self.close_configuration = rospy.get_param(
            "~close_configuration", None)
        if not self.close_configuration:
            rospy.logerr(
                "No close configuration values given in param: ~close_configuration")
            exit(1)
        if len(self.close_configuration) != len(self.real_joint_names):
            rospy.logerr(
                "The close configuration values defined in param: ~close_configuration"
                ", should be of same length as that of joints in param: ~real_joint_names")
            exit(1)

        # This node Dynamic params
        self.ddr = DDynamicReconfigure(
            self.controller_name + "_grasp_service")

        self.timeout = self.ddr.add_variable("timeout",
                                             "timeout for the closing action",
                                             2.0, 2.0, 5.0)
        self.closing_time = self.ddr.add_variable("closing_time",
                                                  "Time for the closing goal",
                                                  0.2, 0.01, 5.0)
        self.rate = self.ddr.add_variable("rate",
                                          "Rate Hz at which the node closing will do stuff",
                                          25, 10, 50)

        self.pressure_configuration = self.ddr.add_variable("pressure",
                                          "Aditional distance to apply more or less pressure",
                                          0.08, 0.05, 0.2)

        self.ddr.start(self.ddr_cb)
        rospy.loginfo("Initialized dynamic reconfigure on: " +
                      str(rospy.get_name()))

        # Set opening time of gripper
        self.opening_time = 0.02

        # Subscriber to the gripper state
        self.state_sub = rospy.Subscriber('/' + self.controller_name + '_controller/state',
                                          JointTrajectoryControllerState,
                                          self.state_cb,
                                          queue_size=1)
        rospy.loginfo("Subscribed to topic: " +
                      str(self.state_sub.resolved_name))

        # Publisher on the gripper command topic
        self.cmd_pub = rospy.Publisher('/' + self.controller_name + '_controller/command',
                                       JointTrajectory,
                                       queue_size=1)
        rospy.loginfo("Publishing to topic: " +
                      str(self.cmd_pub.resolved_name))

        # Grasping service to offer
        self.grasp_srv = rospy.Service('/' + self.controller_name + '_controller/grasp',
                                       Empty,
                                       self.grasp_cb)
        rospy.loginfo("Offering grasp service on: " +
                      str(self.grasp_srv.resolved_name))
        rospy.loginfo("Done initializing Gripper Grasp Service!")

        # Release service to offer
        self.release_srv = rospy.Service('/' + self.controller_name + '_controller/release',
                                         Empty,
                                         self.release_cb)
        rospy.loginfo("Offering release service on: " +
                      str(self.release_srv.resolved_name))
        rospy.loginfo("Done initializing Gripper Release Service!")

        self.pub_js = rospy.Publisher(
            "{}/is_grasped".format(self.controller_name), Bool, queue_size=1)

    def ddr_cb(self, config, level):
        rospy.loginfo("timeout" + str(config['timeout']) + "closing_time" + str(config['closing_time']))
        if config['timeout'] - 2 <= config['closing_time']:
            self.timeout = config['closing_time'] + 2
            config['timeout'] = config['closing_time'] + 2
        else:
            self.timeout = config['timeout']
        self.closing_time = config['closing_time']
        self.pressure_configuration = config['pressure']
        self.rate = config['rate']
        return config

    def state_cb(self, data):

        self.last_state = data
        if self.on_optimal_close is True:
            self.is_grasped_msg.data = True
            if self.last_state.error.positions[0] <= 0.01:
                self.is_grasped_msg.data = False
                self.on_optimal_close = False
        else:
            self.is_grasped_msg.data = False
        self.pub_js.publish(self.is_grasped_msg)


    def check_is_grasped(self):

        if self.last_state.error.positions[0] >= 0.03:
            rospy.sleep(1)
        if self.last_state.error.positions[0] >= 0.03:
            return True
        return False


    def grasp_cb(self, req):
        rospy.logdebug("Received grasp request!")
        # From wherever we are close gripper
        self.on_optimal_open = False
        # Keep closing until the error of the state reaches
        # max_position_error on any of the gripper joints
        # or we reach timeout
        initial_time = rospy.Time.now()
        r = rospy.Rate(self.rate)
        closing_amount = self.close_configuration
        # Initial command, wait for it to do something
        rospy.loginfo("Closing: " + str(closing_amount))
        self.send_joint_trajectory(
            closing_amount, self.closing_time, self.on_optimal_close)
        while not rospy.is_shutdown() and \
                  (rospy.Time.now() - initial_time) < rospy.Duration(self.timeout) and \
                  not self.on_optimal_close and self.last_state:
            if self.check_is_grasped():
                closing_amount = [self.last_state.actual.positions[0]+self.pressure_configuration]
                self.on_optimal_close = True
                rospy.loginfo("Closing: " + str(closing_amount))
                self.send_joint_trajectory(
                    closing_amount, self.closing_time, self.on_optimal_close)
            r.sleep()

        return EmptyResponse()

    def release_cb(self, req):
        rospy.logdebug("Received release request!")
        # From wherever we are opening gripper
        self.on_optimal_close = False

        open_amount = [0.0] * len(self.real_joint_names)
        # Initial command, wait for it to do something
        if self.on_optimal_open is False:
            rospy.loginfo("Opening: " + str(open_amount))
            self.send_joint_trajectory(
                open_amount, self.opening_time, self.on_optimal_open)
            self.on_optimal_close = False
            self.on_optimal_open = True
            rospy.sleep(self.opening_time)

        return EmptyResponse()

    def send_joint_trajectory(self, joint_positions, execution_time, goal_position_reached):
        jt = JointTrajectory()
        jt.joint_names = self.real_joint_names
        jt.header.stamp = rospy.Time.now()
        p = JointTrajectoryPoint()
        p.positions = joint_positions

        # on_optimal_position can be either for on_optimal_close or on_optimal_open
        if goal_position_reached == True:
            # Duration after grasping
            p.time_from_start = rospy.Duration(0.2)
        else:
            # Duration of the grasping
            p.time_from_start = rospy.Duration(execution_time)
        jt.points.append(p)

        self.cmd_pub.publish(jt)

if __name__ == '__main__':
    rospy.init_node('gripper_grasping')
    gg = GripperGrasp()
    rospy.spin()

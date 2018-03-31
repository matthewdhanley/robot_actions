#! /usr/bin/env python
import rospy
import actionlib
import robot_actions.msg  # imports several messages for sending goals, reciving feedback, etc.
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
import time


class RobotAction(object):
    # create messages that are used to publish feedback and result
    # underscore indicates that it is a "privite variable," not enforced by language
    #   but instead because python programmers are "consenting adults"
    _feedback = robot_actions.msg.move_baseFeedback()
    _result = robot_actions.msg.move_baseResult()

    def __init__(self, name):
        # passed in a name for the namespace, making it available to the class
        self._action_name = name

        # creating an Action Server (as)
        # execute_cb is an optional arguement specifying the execute callback
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                robot_actions.msg.move_baseAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)

        # subscribe to something
        rospy.Subscriber('/tag_detections_pose', PoseArray, self.subscribe_cb)

        # publishing
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # constants
        self.max_scalar = 2
        self.max_speed = 100.0
        self.kp = 15  # proportional constant
        self.max_time = 60  # time to get to goal

        # self.time_limit = 500  # time limit to get to target
        rospy.loginfo("RobotAction Server Init Complete.")

        # starting the action server
        self._as.start()

    def subscribe_cb(self, data):
        if self._as.is_preempt_requested():
            return
        feedback_tmp = Point()
        try:
            feedback_tmp.x = data.poses[0].position.x
            feedback_tmp.y = data.poses[0].position.z
        except IndexError:
            feedback_tmp.x = -1
            feedback_tmp.y = -1

        self._feedback.location = feedback_tmp

    def execute_cb(self, goal):
        # helper vars
        r = rospy.Rate(1)
        success = True

        # publish info to console for user
        rospy.loginfo("Starting Action move_base")
        rospy.loginfo("goal: %s" % goal.location)

        start_time = time.time()
        while self._feedback.location.y < goal.location.y and success:

            # we want to check if the requested action has been canceled or "preempted"
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            cmd_vel = Twist()
            if self._feedback.location.y == -1:
                speed = 0
            else:
                # move the robot accordingly
                speed = (goal.location.y - self._feedback.location.y) * self.kp

            # apply saturation to speed
            if speed > self.max_speed:
                speed = self.max_speed
            elif speed < -self.max_speed:
                speed = -self.max_speed

            cmd_vel.linear.x = speed / 100.0 * self.max_scalar

            # publish to make robot go
            self.pub.publish(cmd_vel)

            # publish feedback
            self._as.publish_feedback(self._feedback)
            r.sleep()
            cur_time = time.time()

            if cur_time - start_time > self.max_time:
                success = False

        if success:
            self._result.location = self._feedback.location
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted()
            rospy.loginfo('Aborted.')


if __name__ == '__main__':
    rospy.init_node('move_base')
    server = RobotAction(rospy.get_name())
    rospy.spin()
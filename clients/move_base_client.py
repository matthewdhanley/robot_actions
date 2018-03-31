#! /usr/bin/env python
from __future__ import print_function
import rospy
import sys
from geometry_msgs.msg import Point

import actionlib
import robot_actions.msg  # imports several messages for sending goals, reciving feedback, etc.

def move_base_client():
    client = actionlib.SimpleActionClient('move_base',
                                          robot_actions.msg.move_baseAction)

    client.wait_for_server()

    goal_tmp = Point()
    goal_tmp.x = 1
    goal_tmp.y = 10

    goal = robot_actions.msg.move_baseGoal(location=goal_tmp)

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('move_base_client')
        result = move_base_client()

        print(result.location)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
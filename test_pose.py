# Jackal navigate to goal
# ref: https://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal

#!/usr/bin/env python

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))
import rospy
from jackal_agent import JackalAgent
import math

class TestPose(JackalAgent):
    def __init__(self):
        """ initialize GoToGoal object """
        super(TestPose, self).__init__()

    def test_pose(self):
        """ test relative pose of Jackal robot """
        print("x: {}, y: {}, theta: {}"\
            .format(self.pose.x, self.pose.y, math.degrees(self.pose.theta)))    
    
if __name__ == '__main__':
    try:
        rospy.init_node("jackal_goal", anonymous=True)
        rate = rospy.Rate(10)
        agent = TestPose()
        while not rospy.is_shutdown():
            agent.test_pose()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
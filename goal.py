# Jackal navigate to goal
# ref: https://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal

#!/usr/bin/env python

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))
import rospy
from jackal_agent import JackalAgent

class GoToGoal(JackalAgent):
    def __init__(self):
        """ initialize GoToGoal object """
        super(GoToGoal, self).__init__()

    def go_to_goal(self):
        """ navigate robot to reach goal point """
        mode = int(input("Mode (1 - both, 2 - head first): "))
        print("Input desired goal coordinates")
        goal_x = round(float(input("x: ")), 3)
        goal_y = round(float(input("y: ")), 3)
        self.tol = round(float(input("Tolerance: ")), 3)
        self.set_goal(goal_x, goal_y)
        self.nav_goal(mode)

        if self.goal_dist(self.goal_pose) < self.tol:
            self.stop_bot()
            self.orient_bot()
            print("Goal reached.")

if __name__ == '__main__':
    try:
        rospy.init_node("jackal_goal", anonymous=True)
        agent = GoToGoal()
        agent.go_to_goal()
    except rospy.ROSInterruptException:
        pass
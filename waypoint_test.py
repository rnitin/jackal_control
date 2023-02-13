# Jackal navigate through waypoints
# ref: https://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal

#!/usr/bin/env python

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))
import rospy
import numpy as np
from jackal_agent import JackalAgent

class WaypointNav(JackalAgent):
    def __init__(self):
        """ initialize WaypointNav object """
        super(WaypointNav, self).__init__()
   
    def go_to_wayp(self):
        """ navigate Jackal through waypoints """
        # mode = int(input("Mode (1 - both, 2 - head first): "))
        # n_wayp = int(input("Input number of waypoints: "))
        # wayp = np.zeros((n_wayp, 2))
        # x_str = input("x coordinates of waypoints (separated by ','): ")
        # wayp[:,0] = np.array(list(map(float, x_str.split(','))))
        # y_str = input("y coordinates of waypoints (separated by ','): ")
        # wayp[:,1] = np.array(list(map(float, y_str.split(','))))
        # self.tol = round(float(input("Tolerance: ")), 3)

        mode = 2
        n_wayp = 4
        wayp = np.zeros((n_wayp, 2))
        wayp[:,0] = np.array([1,1,0,0])
        wayp[:,1] = np.array([0,1,1,0])

        for i_wayp in range(n_wayp):
            goal_x = round(wayp[i_wayp,0], 3)
            goal_y = round(wayp[i_wayp,1], 3)
            self.set_goal(goal_x, goal_y)
            self.nav_goal(mode)
            print("Waypoint ", i_wayp+1, " reached.")
            rospy.sleep(0.5)
            self.turn(5*np.pi/2)
            rospy.sleep(0.5)

        if (i_wayp == n_wayp - 1):
            print("All waypoints navigated.")
            self.stop_bot()
            #self.orient_bot()
        else:
            print("Error. ", i_wayp + 1, " waypoints navigated.")

if __name__ == '__main__':
    try:
        rospy.init_node("jackal_wayp", anonymous=True)
        agent = WaypointNav()
        agent.go_to_wayp()
    except rospy.ROSInterruptException:
        pass
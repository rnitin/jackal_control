# Jackal navigate through waypoints
# ref: https://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal

#!/usr/bin/env python

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))
import rospy
import numpy as np
import pickle
from jackal_agent import JackalAgent

class WaypointNav(JackalAgent):
    def __init__(self):
        """ initialize WaypointNav object """
        super(WaypointNav, self).__init__(logging=True)

    def go_to_wayp(self):
        """ navigate Jackal through waypoints """
        mode = int(input("Mode (1 - both, 2 - head first): "))
        n_wayp = 5
        wayp = np.array([[0.,0.],[1.,0.],[1.,1.],[0.,1.],[0.,0.]])
        self.tol= 0.005

        for i_wayp in range(n_wayp):
            goal_x = round(wayp[i_wayp,0], 3)
            goal_y = round(wayp[i_wayp,1], 3)
            self.set_goal(goal_x, goal_y)
            self.nav_goal(mode)
            print("Waypoint ", i_wayp+1, " reached.")

        if (i_wayp == n_wayp - 1):
            print("All waypoints navigated.")
            self.stop_bot()
            self.orient_bot()
            log_path = os.path.join(os.getcwd(), 'src/jackal_control/logs/')
            with open(log_path + 'list_er_lin.pkl', 'wb') as f:
                pickle.dump(self.list_e_lin, f)
            with open(log_path + 'list_er_ang.pkl', 'wb') as f:
                pickle.dump(self.list_e_ang, f)
        else:
            print("Error. ", i_wayp + 1, " waypoints navigated.")

if __name__ == '__main__':
    try:
        rospy.init_node("jackal_wayp_test", anonymous=True)
        agent = WaypointNav()
        agent.go_to_wayp()
    except rospy.ROSInterruptException:
        pass

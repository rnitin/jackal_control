import numpy as np
import math
import time

class Pose:
    """ class to represent 2D pose of mobile robot """
    def __init__(self, x=0, y=0, theta=0):
        """ initialize Pose object """
        self.x = x
        self.y = y
        self.theta = theta

    def update_pose(self, x=0, y=0, theta=0):
        """ update pose to new input """
        self.x = x
        self.y = y
        self.theta = theta


class PID:
    """ class that implements a PID controller """
    def __init__(self, k_p=0, k_i=0, k_d=0):
        """ initialize PID object """
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.reset()

    def reset(self):
        """ reset stored error and time values """
        self.e_sum = 0.0
        self.e_prev = 0.0
        self.t_prev = time.time()

    def get_control(self, e):
        """ obtain control input corresponding to the current error """
        t_now = time.time()
        t_del = t_now - self.t_prev
        self.t_prev = t_now
        self.e_sum += e*t_del
        e_del = e - self.e_prev
        self.e_prev = e

        control = self.k_p*e + self.k_i*self.e_sum + self.k_d*e_del/t_del
        return control


class RobotAgent:
    """ class representing a 2D mobile robot """
    def __init__(self, MAX_V_LIN, MAX_V_ANG):
        self.pose = Pose()
        self.goal_pose = Pose()
        self.MAX_V_LIN = MAX_V_LIN
        self.MAX_V_ANG = MAX_V_ANG

    def goal_dist(self, goal_pose):
        """ compute L2 norm to goal from current pose """
        return np.linalg.norm([goal_pose.x - self.pose.x, \
            goal_pose.y - self.pose.y])
    
    def goal_head(self, goal_pose):
        """ compute heading to goal from current pose """
        e_ang = math.atan2(goal_pose.y - self.pose.y, \
            goal_pose.x - self.pose.x) - self.pose.theta
        return wrap_angle(e_ang)
    
    def clip_vel(self, v_lin, v_ang, thresh=1.0):
        """ clip velocity commands sent to the bot"""
        max_lin = thresh * self.MAX_V_LIN
        max_ang = thresh * self.MAX_V_ANG
        if v_lin > max_lin:
            v_lin = max_lin
        elif v_lin < -1*max_lin:
            v_lin = -1*max_lin
        if v_ang > max_ang:
            v_ang = max_ang
        elif v_ang < -1*max_ang:
            v_ang = -1*max_ang
        return v_lin, v_ang


def wrap_angle(theta):
    """ wraps angles to the range [-PI,PI] """
    while theta > math.pi:
        theta = theta - 2*math.pi
    while theta < -1*math.pi:
        theta = theta + 2*math.pi
    return theta
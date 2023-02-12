import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
import transforms3d as t3d
import numpy as np
import math
from utils import RobotAgent, PID
from utils import wrap_angle

class JackalAgent(RobotAgent):
    """ class representing a Jackal Robot """
    
    def __init__(self, k=np.array([[0.85, 0., 0.],[1.25, 0., 0.]]), \
    logging=False):
        """ initialize JackalAgent object """
        self.wait_sub = True # flag to wait for subscriber init
        super(JackalAgent, self).__init__(MAX_V_LIN=0.5, MAX_V_ANG=1.0)
        
        rospy

        self.rate = rospy.Rate(100)
        tfBuffer_orig = tf2_ros.Buffer()
        listener_orig = tf2_ros.TransformListener(tfBuffer_orig)
        receive_trans = False
        while not receive_trans:
            try:
                trans = tfBuffer_orig.lookup_transform("base_link", "odom", rospy.Time())
                receive_trans = True
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue

        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = "odom"
        static_transformStamped.transform.translation.x = float(0)
        static_transformStamped.transform.translation.y = float(0)
        static_transformStamped.transform.translation.z = float(0)
        static_transformStamped.transform.rotation.x = trans.transform.rotation.x
        static_transformStamped.transform.rotation.y = trans.transform.rotation.y
        static_transformStamped.transform.rotation.z = trans.transform.rotation.z
        static_transformStamped.transform.rotation.w = trans.transform.rotation.w
        # static_transformStamped.transform.rotation.x = float(0)
        # static_transformStamped.transform.rotation.y = float(0)
        # static_transformStamped.transform.rotation.z = float(0)
        # static_transformStamped.transform.rotation.w = float(1)

        broadcaster.sendTransform(static_transformStamped)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/odometry/filtered", Odometry, self.update_pose)
        self.rate = rospy.Rate(10)
        self.PID_lin = PID(k_p=k[0,0], k_i=k[0,1], k_d=k[0,2])
        self.PID_ang = PID(k_p=k[1,0], k_i=k[1,1], k_d=k[1,2])
        self.tol = 0.01
        self.logging = logging
        if self.logging is True:
            self.log_e_lin = []
            self.log_e_ang = []
            self.list_e_lin = []
            self.list_e_ang = []
        while self.wait_sub: # wait til subscriber receives msg
            rospy.sleep(0.01)

    def set_goal(self, goal_x=0, goal_y=0, goal_theta=0):
        """ set goal of the robot """
        self.goal_pose.x = goal_x
        self.goal_pose.y = goal_y
        self.goal_pose.theta = goal_theta

    def update_pose(self, msg):
        """ callback function from subscriber to update pose """
        if self.wait_sub:
            self.wait_sub = False

        tfBuffer_world = tf2_ros.Buffer()
        listener_world = tf2_ros.TransformListener(tfBuffer_world)
        receive_trans = False
        while not receive_trans:
            try:
                trans = tfBuffer_world.lookup_transform("world", "base_link", rospy.Time())
                receive_trans = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue

        self.pose.x = round(trans.transform.translation.x, 3)
        self.pose.y = round(trans.transform.translation.y, 3)
        # self.pose.z = round(trans.transform.tranlation.z, 3)
        o_x = trans.transform.rotation.x
        o_y = trans.transform.rotation.y
        o_z = trans.transform.rotation.z
        o_w = trans.transform.rotation.w
        q = [o_x, o_y, o_z, o_w]
        E = t3d.euler.quat2euler(q)
        self.pose.theta = round(wrap_angle(E[0]), 3)

        #     self.init_pose.x = round(msg.pose.pose.position.x, 3)
        #     self.init_pose.y = round(msg.pose.pose.position.y, 3)
        #     o_x = msg.pose.pose.orientation.x
        #     o_y = msg.pose.pose.orientation.y
        #     o_z = msg.pose.pose.orientation.z
        #     o_w = msg.pose.pose.orientation.w
        #     q = [o_x, o_y, o_z, o_w]
        #     E = t3d.euler.quat2euler(q)
        #     self.init_pose.theta = round(wrap_angle(E[0]), 3)

        #     # global frame transformation
        #     tf_x = 0 - self.init_pose.x
        #     tf_y = 0 - self.init_pose.y
        #     tf_theta = wrap_angle(0 - self.init_pose.theta)
        #     self.tf_mat = np.array([
        #                 [math.cos(tf_theta), -math.sin(tf_theta), tf_x],
        #                 [math.sin(tf_theta), math.cos(tf_theta), tf_y],
        #                 [0., 0., 1.]]
        #             )      

        # else:
        #     x = round(msg.pose.pose.position.x, 3)
        #     y = round(msg.pose.pose.position.y, 3)
        #     o_x = msg.pose.pose.orientation.x
        #     o_y = msg.pose.pose.orientation.y
        #     o_z = msg.pose.pose.orientation.z
        #     o_w = msg.pose.pose.orientation.w
        #     q = [o_x, o_y, o_z, o_w]
        #     E = t3d.euler.quat2euler(q)
        #     theta = round(wrap_angle(E[0]), 3)

        #     # relative pose
        #     x -= self.init_pose.x
        #     y -= self.init_pose.y
        #     theta = wrap_angle(theta - self.init_pose.theta)

        #     # global pose
        #     global_pos = np.matmul(self.tf_mat, np.array([[x], [y], [1]]))
        #     self.pose.x = global_pos[0]
        #     self.pose.y = global_pos[1]
        #     self.pose.theta = wrap_angle(theta - self.init_pose.theta)


    def stop_bot(self):
        """ stop bot by sending 0 velocity commands"""
        v_msg = Twist()
        v_msg.linear.x = 0
        v_msg.angular.z = 0
        self.pub.publish(v_msg)

    def orient_bot(self, ANG_THRESH=0.05, target=0):
        """ reset orientation of bot """
        self.PID_ang.reset()
        e_ang = wrap_angle(target - self.pose.theta)
        while abs(e_ang) > ANG_THRESH:
            v_ang = self.PID_ang.get_control(e_ang)
            v_msg = Twist()
            v_msg.linear.x = 0
            v_msg.angular.z = v_ang
            self.pub.publish(v_msg)
            self.rate.sleep()
            e_ang = wrap_angle(target - self.pose.theta)
        self.stop_bot()

    def get_v_la(self):
        """ compute v_lin and v_ang towards goal """
        e_lin = self.goal_dist(self.goal_pose)
        e_ang = self.goal_head(self.goal_pose)
        v_lin = self.PID_lin.get_control(e_lin)
        v_ang = self.PID_ang.get_control(e_ang)
        v_lin, v_ang = self.clip_vel(v_lin, v_ang)
        v_msg = Twist()
        v_msg.linear.x = v_lin
        v_msg.angular.z = v_ang
        
        if self.logging is True:
            self.log_e_lin.append(e_lin)
            self.log_e_ang.append(e_ang)
        
        return v_msg

    def get_v_a(self):
        """ compute v_ang towards goal """
        e_lin = self.goal_dist(self.goal_pose)
        e_ang = self.goal_head(self.goal_pose)
        v_lin = 0
        v_ang = self.PID_ang.get_control(e_ang)
        v_lin, v_ang = self.clip_vel(v_lin, v_ang)
        v_msg = Twist()
        v_msg.linear.x = v_lin
        v_msg.angular.z = v_ang
        
        if self.logging is True:
            self.log_e_lin.append(e_lin)
            self.log_e_ang.append(e_ang)

        return v_msg

    def nav_goal(self, mode=1, ANG_THRESH=0.1):
        if mode == 1:            
            """ navigate to goal by fixing both e_lin and e_ang simultaneously """
            while self.goal_dist(self.goal_pose) >= self.tol \
            and not rospy.is_shutdown():
                v_msg = self.get_v_la()
                self.pub.publish(v_msg)
                self.rate.sleep()
        else:
            """ navigate to goal by fixing e_ang first """
            self.PID_ang.reset()
            v_msg = self.get_v_a()
            while abs(v_msg.angular.z) >= ANG_THRESH \
            and self.goal_dist(self.goal_pose) >= self.tol \
            and not rospy.is_shutdown():
                self.pub.publish(v_msg)
                self.rate.sleep()
                v_msg = self.get_v_a()

            self.PID_ang.reset()
            self.PID_lin.reset()
            while self.goal_dist(self.goal_pose) >= self.tol \
            and not rospy.is_shutdown():
                v_msg = self.get_v_la()
                self.pub.publish(v_msg)
                self.rate.sleep()
        
        if self.logging is True:
            self.list_e_lin.append(self.log_e_lin)
            self.list_e_ang.append(self.log_e_ang)
            self.log_e_lin = []
            self.log_e_ang = []
            

    def turn(self, angle, ANG_THRESH=0.2):
        """ turn counter-clockwise for a given (positive) angle """
        rot_angle = 0
        init_angle = self.pose.theta
        print("a {}, i {}, r {}".format(math.degrees(angle), 
                math.degrees(init_angle), math.degrees(rot_angle)))
        while abs (rot_angle - angle) > ANG_THRESH \
        and not rospy.is_shutdown():
            v_msg = Twist()
            v_msg.linear.x = 0
            v_msg.angular.z = self.MAX_V_ANG / 2
            self.pub.publish(v_msg)
            self.rate.sleep()

            pres_angle = self.pose.theta
            # we are assuming max rotation in one step is less than 360 deg
            if init_angle > pres_angle: 
                pres_rot_angle = 2*np.pi + pres_angle - init_angle
            else:
                pres_rot_angle = pres_angle - init_angle
            
            rot_angle += pres_rot_angle
            init_angle = pres_angle
            print(math.degrees(pres_angle), math.degrees(rot_angle))
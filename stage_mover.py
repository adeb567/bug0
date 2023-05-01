import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import tf.transformations as transform
import math, sys
import numpy as np


laser_data = {"left": 0, "straight": 0, "right":0, 'all':0}
position = {"rob_x": 0, "rob_y": 0, "rob_z": 0}
dist_threshold = 1
obs_threshold = 0.5
rot_threshold = 0.1


def callback_0(msg):
    laser_data["right"] = np.array(msg.ranges[160:200])
    laser_data["straight"] = np.array(msg.ranges[500:580])
    laser_data["left"] = np.array(msg.ranges[900:940])
    laser_data['all'] = np.array(msg.ranges)


def callback_1(msg):
    position["rob_x"] = msg.pose.pose.position.x
    position["rob_y"] = msg.pose.pose.position.y
    rob_orient = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
    transformed = transform.euler_from_quaternion(rob_orient)
    position["rob_z"] = transformed[2]
    
def get_distance():
    rob_pos = np.array((position["rob_x"], position["rob_y"]))
    goal_pos = np.array((x, y))
    return  np.linalg.norm(rob_pos - goal_pos)

def move_towards_goal():
    while get_distance() > dist_threshold:
        if np.any(laser_data["straight"] < obs_threshold):
            print("end of move towards obstacle")
            break
        #Rotate to target
        rotate_to_target = math.atan2(y - position["rob_y"], x - position["rob_x"])
        if abs(position["rob_z"] - rotate_to_target) >= rot_threshold:
            com.angular.z = 0.2
            com.linear.x = 0
            cmd_vel_pub.publish(com)
            print("in facing goal")
        elif np.all(laser_data["straight"] > obs_threshold):
            #go straight towards goal until obstracle
            com.linear.x = 0.4
            com.angular.z = 0
            cmd_vel_pub.publish(com)
            print("in reaching obstacle/goal")
        rospy.sleep(0.1)


def stay_at_wall():
    #if np.any(laser_data["right"] > obs_threshold + 1.5):
    rotate_to_target = math.atan2(y - position["rob_y"], x - position["rob_x"])
    direction = (rotate_to_target - position["rob_z"]) * 180/math.pi
    check_index = int(540 + direction * 4)
    if 180 < check_index < 920:
        if laser_data['all'][check_index] > obs_threshold + 2:
            return False
    return True


def follow_wall():
    if np.any(laser_data["straight"] <= obs_threshold + 1.5):
        com.linear.x = 0
        com.angular.z = 0.2
        cmd_vel_pub.publish(com)
        print("in turning parallel to wall")

    elif np.all(laser_data["right"] > obs_threshold + 1.5):
        com.angular.z = -0.2
        com.linear.x = 0
        cmd_vel_pub.publish(com)
        print("in turning right at wall")

    elif np.all(laser_data["straight"] > obs_threshold + 0.5):
        com.angular.z = 0
        com.linear.x = 0.2
        cmd_vel_pub.publish(com)
        print("in moving along wall")
    else:
        com.angular.z = 0.2
        com.linear.x = 0
        cmd_vel_pub.publish(com)


def bug0():
    while get_distance() > dist_threshold:
        move_towards_goal()

        while stay_at_wall():
            follow_wall()
            rospy.sleep(0.1)
        
        for i in range(100):
            com.angular.z = 0
            com.linear.x = 0.1
            cmd_vel_pub.publish(com)
            print("prevent crash")
            rospy.sleep(0.1)

    print("GOAL REACHED")

if __name__ == "__main__":

    x = float(sys.argv[1])
    y = float(sys.argv[2])

    rospy.init_node("stage_mover", anonymous=True)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    scan_sub = rospy.Subscriber('/base_scan', LaserScan, callback_0)
    pos_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, callback_1)
    rospy.sleep(1)
    com = Twist()

    bug0()
    
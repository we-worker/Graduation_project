#!/usr/bin/python
# -*- coding: UTF-8 -*-

from actionlib.action_client import GoalManager
import rospy 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math

from geometry_msgs.msg import PoseStamped

def pose_callback(msg):
    global current_pos
    current_pos= msg.pose
    # rospy.loginfo("Robot position: x=%f, y=%f, z=%f" % (current_pos.position.x, current_pos.position.y, current_pos.orientation.z))



def calculate_distance(goal, current_position):
    xy_distance = math.sqrt((goal.target_pose.pose.position.x - current_position.position.x)**2 
                     + (goal.target_pose.pose.position.y - current_position.position.y)**2)
    sign = 1 if current_position.orientation.w >= 0 else -1
    yaw_distance = abs(goal.target_pose.pose.orientation.z - sign * current_position.orientation.z)
    return xy_distance, yaw_distance

def send_next_goal(client, goals, current_goal_index):
    if current_goal_index < len(goals):
        goal = goals[current_goal_index]
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal)
        rospy.loginfo("Send NO. %s Goal !!!" % str(current_goal_index))
        return True
    else:
        rospy.loginfo("All goals have been sent.")
        return False

def send_goals_python():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    #定义四个发送目标点的对象
    goals = [MoveBaseGoal() for _ in range(2)]
    # 
# 
    # 初始化四个目标点在 map 坐标系下的坐标,pose.x,pose.y,#orientation.z,orientation.w   sin((90/2)*pi/180),cos((90/2)*pi/180)
    # =0.707106,cos((90/2)*pi/180)=0.707106
    goals_data = [
        (-1.51737,-0.86229,math.sin((-90/2)*math.pi/180),math.cos((-90/2)*math.pi/180)),
        (-2.421395,-0.981683, math.sin((-90/2)*math.pi/180),math.cos((-90/2)*math.pi/180))
        # (3, 2, 0.7021705987793959),
        # (3.75, 2, 0.7021705987793959,)
        # (-1.51737,-0.86229,),
    ]
    
    for goal, data in zip(goals, goals_data):
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w = data
        # =math.sqrt(1-goal.target_pose.pose.orientation.z**2)     #orientation.z,orientation.w
    
    current_goal_index = 0
    while True:      
        if send_next_goal(client, goals, current_goal_index):
            xy_distance, yaw_distance = calculate_distance(goals[current_goal_index], current_pos)
            print(xy_distance,yaw_distance)
            while xy_distance > 0.2 or yaw_distance > 0.1:  # 0.1 is the tolerance
                rospy.sleep(1)  # sleep for a while before checking again
                xy_distance, yaw_distance = calculate_distance(goals[current_goal_index], current_pos)
                print(xy_distance,yaw_distance)
            rospy.loginfo("The NO. %s Goal achieved success !!!" % str(current_goal_index))
            current_goal_index += 1
        else:
            break
    return "Mission Finished."

if __name__ == '__main__':
    rospy.init_node('send_goals_python',anonymous=True)
    current_pos= None
    rospy.Subscriber("/tracked_pose", PoseStamped, pose_callback)
    result = send_goals_python()
    rospy.loginfo(result)
    rospy.spin()


# wait = client.wait_for_result(rospy.Duration.from_sec(8.0))  # 发送完毕目标点之后，根据action 的机制，等待反馈执行的状态，等待时长是：30 s.
        

#! /usr/bin/env python

#https://docs.google.com/presentation/d/1YD-KUK0olqWNCdKe9I6reWIyJE-Cx3kiv5WSWRZ_ZIQ/edit?usp=sharing
import rospy
import sys
import copy
import cv2
import random
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import numpy as np

from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError


class Npm_1(object):

    def __init__(self):

        '''
         ______________________________________________________________
        |
        |Description
        |------------
        | - Class Constructor method.
        |
        |..............................................................
        |
        | Parameters
        | ----------
        |   None
        |
        |..............................................................
        |
        | Constants/variables
        | -------------------
        | * self._planning_group - Name of planning group
        | * self._robot_ns - Name of robot
        | * self._commander - moveit commander
        | * self._scene - name of planning scene.
        | * self._display_trajectory_publisher - publishes data about
        |                                        the robot's trajectory
        | * self._exectute_trajectory_client - action client to execute
        |                                      trajectory.
        |
        | Objects
        | -------
        | self._robot
        |          - robot in planning scene.
        |
        | self._group
        |          - move_group commander to handle trajectories.
        |______________________________________________________________
        '''

        #Constants
        self.planning_group_1 = "planner_1"
        self.planning_group_2 = "planner_2"
        self.planning_group_3 = "planner_3"

        #Initialize Node and Moveit Commander.
        moveit_commander.roscpp_initialize(sys.argv)

        #Instantiates
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group_1 = moveit_commander.MoveGroupCommander(self.planning_group_1)
        self._group_2 = moveit_commander.MoveGroupCommander(self.planning_group_2)
        self._group_3 = moveit_commander.MoveGroupCommander(self.planning_group_3)
        self.trajectory_client = actionlib.SimpleActionClient('execute_trajectory',moveit_msgs.msg.ExecuteTrajectoryAction)
        self.trajectory_client.wait_for_server()

        #Basic Info
        self.planning_frame = self._group_1.get_planning_frame()
        self.eef_link = self._group_1.get_end_effector_link()
        self.group_names = self._robot.get_group_names()

        #Initial Setup
        self._group_1.set_goal_tolerance(0.01)
        self._group_1.set_num_planning_attempts(3)
        self._group_2.set_goal_tolerance(0.001)
        self._group_2.set_num_planning_attempts(3)
        self._group_3.set_goal_tolerance(0.01)
        self._group_3.set_num_planning_attempts(3)

        #Joint Angles for arm
        self.BLUE_BOX = [2.4,-1.2]
        self.RED_BOX = [2.2,-1.23]
        self.GREEN_BOX = [1.95,-1.2]

        #Variables
        self.pose = ""
        self.position_tried = ""
        self.arm_velocity = 2.0

        #Files
        self.JOINT_LIMITS_1 = '/robot_description_planning/joint_limits/'
        self.JOINT_LIMITS_2 = '/true_planning/joint_limits/'

    def go_to_defined_pose(self,pose_name,group_name):

        '''
         ______________________________________________________________
        |
        |Description
        |------------
        | - Makes the arm go the predefined positions according to the
        |   Robot's SRDF.For this robot theese positions are home,
        |   red_bin_pose,blue_bin_pose, green_bin_pose and pick.
        |
        |..............................................................
        | Parameters
        | ----------
        |   pose_name:str
        |        - The position name like (home,pick...) are given as an
        |          argument.
        |
        |..............................................................
        | Returns
        | -------
        | None
        |______________________________________________________________
        '''

        if group_name == self.planning_group_1:
            rospy.loginfo('\033[94m' + "Going to Pose: {}".format(pose_name) + '\033[0m')
            self._group_1.set_named_target(pose_name) 
            plan = self._group_1.plan()
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
            goal.trajectory = plan
            self.trajectory_client.send_goal(goal)
            self.trajectory_client.wait_for_result()
            rospy.loginfo('\033[94m' + "Now at Pose: {}".format(pose_name) + '\033[0m')

        elif group_name == self.planning_group_2:
            rospy.loginfo('\033[94m' + "Going to Pose: {}".format(pose_name) + '\033[0m')
            self._group_2.set_named_target(pose_name) 
            plan = self._group_2.plan()
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
            goal.trajectory = plan
            self.trajectory_client.send_goal(goal)
            self.trajectory_client.wait_for_result()
            rospy.loginfo('\033[94m' + "Now at Pose: {}".format(pose_name) + '\033[0m')

        elif group_name == self.planning_group_3:
            rospy.loginfo('\033[94m' + "Going to Pose: {}".format(pose_name) + '\033[0m')
            self._group_3.set_named_target(pose_name) 
            plan = self._group_3.plan()
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
            goal.trajectory = plan
            self.trajectory_client.send_goal(goal)
            self.trajectory_client.wait_for_result()
            rospy.loginfo('\033[94m' + "Now at Pose: {}".format(pose_name) + '\033[0m')

    def go_to_pose(self,pose):
        '''
         ______________________________________________________________
        |
        |Description
        |------------
        | - Makes the arm go the custom positions in the simulation
        |   world. These position can be given through geometry_msgs.msg
        |   message format.
        |
        |..............................................................
        | Parameters
        | ----------
        |   cordinates:list >> [x,y,z]
        |        - A list of cartesian cordinates is given as an argument.
        | 
        |
        |..............................................................
        | Returns
        | -------
        | None
        |______________________________________________________________
        '''

        pose_values = self._group_1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        
        self._group_1.set_pose_target(pose) 
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group_1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group_1.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if flag_plan == True:
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
        

    def set_joint_angles(self, arg_list_joint_angles,flag_plan = False):
        '''This function will be used to set joint angles of UR5 Arm'''

        list_joint_values = self._group_1.get_current_joint_values()
        #rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        #rospy.loginfo(list_joint_values)

        while(flag_plan != True):
            self._group_1.set_joint_value_target(arg_list_joint_angles)
            self._group_1.plan()
            flag_plan = self._group_1.go(wait=True)

        list_joint_values = self._group_1.get_current_joint_values()
        #rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        #rospy.loginfo(list_joint_values)

        pose_values = self._group_1.get_current_pose().pose
        #rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        #rospy.loginfo(pose_values)

        if flag_plan == True:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def set_pusher(self,joint_angles):

        self._group_2.set_joint_value_target(joint_angles)
        self._group_2.plan()
        flag_plan = self._group_2.go(wait=True)


    def arrive_at_sorting_loc(self):
        robot.go_to_defined_pose('default','planner_2')
        robot.set_joint_angles([0.0,2.0])
        robot.set_joint_angles([0.7,1.7])
 

    def show_poses(self):
        self.go_to_defined_pose("up",'planner_2')
        rospy.sleep(2)
        self.go_to_defined_pose("table",'planner_1')
        rospy.sleep(2)
        self.go_to_defined_pose("30_degree_push",'planner_3')
        rospy.sleep(2)
        self.go_to_defined_pose("90_degree_push",'planner_3')

        
    def __del__(self):
        print("Npm died")


class Camera(object):

    def __init__(self):

        self.image = None
        self.px = None
        self.py = None

        self.x = 100
        self.y = 390
        self.w = 50
        self.h = 600

        #Camera Caliberation Params
        self.ROBOT_X = 0.0
        self.ROBOT_Y = -0.6
        self.ROBOT_Z = 0.0

        self.TABLE_X = 0.0
        self.TABLE_Y = 0.0
        self.TABLE_Z = 0.35

        self.ROBOT_TABLE_X = abs(self.TABLE_X - self.ROBOT_X)
        self.ROBOT_TABLE_Y = abs(self.TABLE_Y - self.ROBOT_Y)
        self.ROBOT_TABLE_Z = abs(self.TABLE_Z - self.ROBOT_Z)

        self.R = [[1,0,0],
                  [0,1,0],
                  [0,0,1]]
        self.D = [[self.ROBOT_TABLE_X,self.ROBOT_TABLE_Y,]]

        self.CM_TO_PIXEL = 11.3/640.0

        self.bridge = CvBridge()

        
    def get_image(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,"passthrough")
        return self.image


    def show_image(self,image):
        cv2.imshow('Image',image)

    def get_pixel_cordinates(self,thresh_image,h,w):

        row_sum = np.matrix(np.sum(thresh_image,0))
        row_num = np.matrix(np.arange(w))
        row_mul = np.multiply(row_sum,row_num)
        row_t = np.sum(row_mul)
        row_t_f = np.sum(np.sum(thresh_image))
        self.px = row_t/row_t_f

        column_sum = np.matrix(np.sum(thresh_image,0))
        column_num = np.matrix(np.arange(w))
        column_mul = np.multiply(column_sum,column_num)
        column_t = np.sum(column_mul)
        column_t_f = np.sum(np.sum(thresh_image))
        self.py = column_t/column_t_f

        return self.px, self.py 


    def detect_red(self,img):
        pass
        
    def crop_image(self,image):
        image = image[self.x:self.y, self.w:self.h]
        return image

    def convert_camera_to_robot_frame(self,px,py,R):
        pass


class NpmPusher(Npm_1):

    def __init__(self):
        super(NpmPusher,self).__init__()
        self.val = None
        self.flattened = False
        self.direction = 'left'

    def sweep(self,angle):
        angle_rad = [(angle * np.pi)/180.0]
        self._group_3.set_joint_value_target(angle_rad)
        self._group_3.go(wait=True)
    
    def push(self,angle,direction):
        angle_rad = [(angle * np.pi)/180.0]
        self._group_3.set_joint_value_target(angle_rad)
        self._group_3.go(wait=True)
        joint_values = self._group_1.get_current_joint_values()
        #print("Joint Values before pusing:",joint_values)

        if direction == 'right':
            self.set_joint_angles([joint_values[0],joint_values[1]-0.8])
            #print("joint values after pushing",self._group_1.get_current_joint_values())

        elif direction == 'left':
            self.set_joint_angles([joint_values[0],joint_values[1]+0.9])
            #print("joint values after pushing",self._group_1.get_current_joint_values())         

        else:
            print("\033[1;31m Unknown Direction" + '\033[0m')


    def increase_level(self,depth):
        flag_plan = False
        while(flag_plan != True):
            joint_value = self._group_2.get_current_joint_values()
            self._group_2.set_joint_value_target([joint_value[0] - depth])
            flag_plan = self._group_2.go(wait=True)

        
        if flag_plan == True:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')


    def decrease_level(self,height):
        flag_plan = False
        while(flag_plan != True):
            joint_value = self._group_2.get_current_joint_values()
            self._group_2.set_joint_value_target([joint_value[0] + height])
            flag_plan = self._group_2.go(wait=True)

        if flag_plan == True:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')


    def flatten_pile(self,angle,depth_image):
        direction = self.get_direction()
        height = len(self.detect_level_curves(depth_image))
        print("height",height)

        if height <= 3:
            self.set_flattened(True)
            print("\033[1;32m Pile has Flattened Successfully" + '\033[0m')

        else:
            d = self.get_depth(height)
            self.go_to_defined_pose('sorting_pose','planner_1')
            self.increase_level(d)
            self.push(angle,direction)
            self.go_to_defined_pose("up","planner_2")
            #self.arrive_at_sorting_loc()

    
    @classmethod
    def detect_level_curves(cls,depth_image):

        LEVEL_CURVES = []
        for i in depth_image:
            depth_val = i

            for j in depth_val:
                val = round(j,2)

                if val not in LEVEL_CURVES:

                    if len(LEVEL_CURVES) > 0:

                        all_curves = True
                        for k in LEVEL_CURVES:
                              diff = abs(k-val)
                              if diff <= 0.025 or val < 0.81:
                                 all_curves = False
                                 break

                        if all_curves == True:
                            LEVEL_CURVES.append(val)

                    else:
                        LEVEL_CURVES.append(val)
        
        print("LEVEL CURVES",LEVEL_CURVES)
        return LEVEL_CURVES

    @staticmethod
    def get_depth(height):

        depth = 0

        if height == 6:
            depth = 0.06
            #depth = 0.12

        elif height == 5:
            depth = 0.085

        elif height == 4:
            depth = 0.12

        else:
            print("\033[1;31m  Aborted:The Robot can sort piles up to 6 level curve pile" + '\033[0m')

        return depth

    def store_value(self,val):
       self.val = val

    def get_value(self):
        return self.val

    def set_flattened(self,value):
        self.flattened = value

    def check_if_flattened(self,subscriber):
        if self.flattened == True:
            subscriber.unregister()

    def get_direction(self):
        if self.direction == 'left':
            self.direction = 'right'

        elif self.direction == 'right':
            self.direction = 'left'

        return self.direction


    def __del__(self):
        print("NPM Pusher Deleted")
       


def callback(msg):
    counter = pusher.get_value()
    image = cam.get_image(msg)
    depth_image = cam.crop_image(image)
    pusher.detect_level_curves(depth_image)
    #pusher.check_if_flattened(sub)
    
    #if counter%10 ==0:
        #pusher.flatten_pile(-30,depth_image)
        #pusher.store_value(counter+1)

    #else:
       #pusher.store_value(counter+1)
       #return


def main():
    
    #Initialize the Node
    rospy.init_node('Npm_1',anonymous=True)
    
    #Declaration of Class Objects
    global robot,cam,pusher,sub
    robot = Npm_1()
    cam = Camera()
    pusher = NpmPusher()

    pusher.store_value(15)

    #pusher.sweep(-30)
    #robot.go_to_defined_pose('up','planner_2') 

    #Create A subscriber
    sub = rospy.Subscriber('/camera/depth/image_raw',Image,callback,queue_size=1)

    rospy.spin()
    

if __name__ == '__main__':
    main()

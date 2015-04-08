#!/usr/bin/env python
import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, conversions
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from baxter_core_msgs.msg import EndEffectorState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import baxter_external_devices
import argparse
from baxter_interface import CameraController, Gripper, Limb
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np
import math
from skimage import segmentation, measure, morphology
import matplotlib.pyplot as plt
import copy
import speech_recognition as sr
import pyaudio
import pyttsx
import time


import pygst
import gst

from darpa_demo.srv import *

def homography():
    dstPoints = np.asarray([[0.55,-0.5],[0.8,-0.5],[0.8,0],[0.55,0]],np.float32)
    srcPoints1 = np.asarray([[482,148,1]],np.float32).T
    srcPoints = np.asarray([[482,148],[406,133],[312,190],[399,222]],np.float32)

    H = cv2.findHomography(srcPoints,dstPoints,0)[0]
    return H

def speak(message):
    string = message
    music_stream_uri = 'http://translate.google.com/translate_tts?tl=en&q='+string
    player = gst.element_factory_make("playbin","player")
    player.set_property('uri',music_stream_uri)
    player.set_state(gst.STATE_PLAYING)

    time.sleep(4)

class track():
    
    def __init__(self, pr_init, pl_init):
        
        self.centerx = 365
        self.centery = 140
        self.coefx = 0.1/(526-369)
        self.coefy = 0.1/(237-90)
        self.count = 0
        self.right = MoveGroupCommander("right_arm")
        self.left = MoveGroupCommander("left_arm")
        self.gripper = Gripper("right")
        self.gripper.calibrate()
        self.gripper.set_moving_force(0.1)
        self.gripper.set_holding_force(0.1)
        self.pr = pr_init
        self.pl = pl_init
        self.angle = 0
        self.position_list = (0.7,-0.3,-0.06,0,0.0,self.angle)
        self.busy = False
        self.blank_image = np.zeros((400,640,3),np.uint8)
        cv2.putText(self.blank_image,"DEMO", (180,200), cv2.FONT_HERSHEY_SIMPLEX, fontScale = 4, color = 255,  thickness = 10 )
        self.movenum = 0
        self.gripper_distance = 100
        self.subscribe_gripper()
        self.robotx = -1
        self.roboty = -1
        self.H = homography()
        self.framenumber = 0
        self.history = np.arange(0,20)*-1
        self.newPosition = True
        self.bowlcamera = None

    def vision_request_right(self, controlID, requestID):
        rospy.wait_for_service('vision_server_vertical')
        try:
            vision_server_req = rospy.ServiceProxy('vision_server_vertical', Vision)
            return vision_server_req(controlID, requestID)
        
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


    def track(self):
        resp = self.vision_request_right(0, 1)
        if resp.response == 1:
            self.busy = False
        

    def subscribe_camera(self):
        
        callback = self.get_img
        camera_image=rospy.Subscriber('cameras/right_hand_camera/image', Image, callback, queue_size = 1)

    def clean_shutdown(self):
        move(self.right, self.pr_init)
        move(self.left, self.pl_init)
        print "Demo finished"
        cv2.imwrite('box_img.png', self.blank_image)
        rospy.signal_shutdown("Done")
        sys.exit()

    def subscribe_gripper(self):
        gripper_msg = rospy.Subscriber('robot/end_effector/right_gripper/state', EndEffectorState, self.gripper_callback)

    def gripper_callback(self,msg):
        self.gripper_distance = msg.position
        

    def move_by_list(self, group, position_list):
        pose = conversions.list_to_pose_stamped(position_list,"base")
        group.set_pose_target(pose)
        group.go()
        rospy.sleep(0.5)
        #if self.gripper_distance < 10: self.errorhandle()

    def errorhandle(self):
        speak("Please give me the tool")
        while (self.gripper_distance < 5):
            self.gripper.open()
            rospy.sleep(3)
            self.gripper.close()
            rospy.sleep(0.5)
    
    
def move(group, pose):
    group.set_pose_target(pose)
    group.go()
    #print (pose.position.x,pose.position.y)
    
def place(group,ep_position,ep_orientation,pose1,pose2):
    
    current_pose = Pose()
    current_pose.position.x = ep_position.x
    current_pose.position.y = ep_position.y
    current_pose.position.z = ep_position.z
    current_pose.orientation.x = ep_orientation.x
    current_pose.orientation.y = ep_orientation.y
    current_pose.orientation.z = ep_orientation.z
    current_pose.orientation.w = ep_orientation.w

    current_pose.position.z += 0.1 

    move(group,current_pose)

    current_pose.position.x -= 0.2
    move(group,current_pose)
    move(group, pose2)
  
def ik():
    ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    poses = {
        
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.8,
                    y=-0.32,
                    z=0.25,
                ),
                orientation=Quaternion(
                    x=-0.007,
                    y=0.697,
                    z=-0.029,
                    w=0.716,
                ),
            ),
        ),
    }

    limb_joints = {

        'right_s0': 0.787,
        'right_s1': -0.635,
        'right_w0': -0.068,
        'right_w1': -1.445,
        'right_w2': 0.0318,
        'right_e0': -0.082,
        'right_e1': 2.027,

    }
 
    ikreq.pose_stamp.append(poses['right'])

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        #limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        #print limb_joints

        arm = Limb('right')

        i = 0 

        while i < 300:
                arm.set_joint_positions(limb_joints)
                i += 1
                rospy.sleep(0.01)

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
 
    return 0

      
            
     
if __name__=='__main__':
    
       
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    rospy.sleep(1)

       
    #pose target 
    global pr
    pr=Pose()
    pr.position.x = -0.2 #0.8
    pr.position.y = -0.6 #-0.3
    pr.position.z = 0.02 # 0.1
    pr.orientation.x = 0
    pr.orientation.y = 1
    pr.orientation.z = 0
    pr.orientation.w = 0
    
    
    pr_init = copy.deepcopy(pr)
    
    global pl
    pl=Pose()
    pl.position.x = 0.21
    pl.position.y = 0.647
    pl.position.z = 0.363
    pl.orientation.x = 0.379
    pl.orientation.y = 0.77
    pl.orientation.z = -0.204
    pl.orientation.w = 0.47
    
    pl_init = copy.deepcopy(pl)
    
    pr_place = Pose()
    pr_place.position.x = 0.315
    pr_place.position.y = -0.855
    pr_place.position.z = -0.1
    pr_place.orientation.x = 0.505
    pr_place.orientation.y = 0.485
    pr_place.orientation.z = -0.497
    pr_place.orientation.w = 0.512

    right = MoveGroupCommander("right_arm")
    left = MoveGroupCommander("left_arm")
    #group.set_random_target()
    #move(right, pr)
    move(left, pl)

    # clean the scene
    scene.remove_world_object("pole")
    scene.remove_world_object("table")
    scene.remove_world_object("bowl")

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    
    p.pose.position.x = 0.4
    p.pose.position.y = -0.9
    p.pose.position.z = -0.4
    scene.add_box("table", p, (1.6, 0.8, 0.4))

    # p.pose.position.x = 0.8
    # p.pose.position.y = 0.0
    # p.pose.position.z = -0.02
    # scene.add_box("bowl", p, (0.8, 0.01, 0.05))
    #scene.remove_world_object("part")
    rospy.sleep(1)
    print ("Standby at initial position")
    
    
    # camera
    # open camera
    # rh_cam=CameraController("right_hand_camera")
    # lh_cam=CameraController("left_hand_camera")
    # h_cam=CameraController("head_camera")
    # rh_cam.open()
    # lh_cam.open()
    # h_cam.open()
    
    # subscribe image
    #camera_image=rospy.Subscriber('cameras/right_hand_camera/image', Image, get_img)

           
      
    # ques = "What do you want me to do?"
    # if not recogVoice(ques):
    #     rospy.signal_shutdown(tracker.clean_shutdown())

    
    pr.position.x = 0.81 #0.8
    pr.position.y = -0.32 #-0.3
    pr.position.z = 0.18 # 0.1
    pr.orientation.x = -0.018
    pr.orientation.y = 0.717
    pr.orientation.z = -0.047
    pr.orientation.w = 0.695

    #move(right, pr)
    #move(right, pr_place)

    

    tracker=track(pr, pl_init)     
    
    tracker.busy = True
    

    done = False
    ik()
    
    #print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        tracker.track()
        if tracker.busy == False:
            tracker.gripper.close()
            rospy.sleep(0.5)
            arm = Limb("right")
            ep_position = arm.endpoint_pose()['position']
            ep_orientation = arm.endpoint_pose()['orientation']
            place(right,ep_position,ep_orientation,pr,pr_place)
            tracker.gripper.open()
            rospy.sleep(0.5)
            pr_place.position.y += 0.2
            move(right,pr_place)
            #move(right,pr_init)
            print "I am done"
            done = True
        
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                print ("Program finished")
                
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                    


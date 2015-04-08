#!/usr/bin/env python
import sys
import rospy
import baxter_external_devices
import argparse
from baxter_interface import CameraController, Gripper, Limb
from sensor_msgs.msg import Image, Range
import cv_bridge
import cv2
import numpy as np
import math
from skimage import segmentation, measure, morphology
import matplotlib.pyplot as plt
import copy
import pyaudio
import pyttsx
import time
import tf

import pygst
import gst

from darpa_demo.srv import *
from baxter_pykdl import baxter_kinematics


def nothing(x):
    nothing

def homography():
    dstPoints = np.asarray([[0.55,-0.5],[0.8,-0.5],[0.8,0],[0.55,0]],np.float32)
    srcPoints1 = np.asarray([[482,148,1]],np.float32).T
    srcPoints = np.asarray([[482,148],[406,133],[312,190],[399,222]],np.float32)

    H = cv2.findHomography(srcPoints,dstPoints,0)[0]
    return H

class vision_server():
    
    def __init__(self):

        rospy.init_node('vision_server_right')
        
        self.pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.busy = False
        self.dx = 0
        self.dy = 0
        self.avg_dx = -1
        self.avg_dy = -1
        self.H = homography()
        self.framenumber = 0
        self.history_x = np.arange(0,10)*-1
        self.history_y = np.arange(0,10)*-1
        self.bowlcamera = None
        self.newPosition = True
        self.foundBowl = 0
        self.centerx = 400
        
        #self.centerx = 250
        self.centery = 150
        #self.centery = 190
        self.coefx = 0.1/(526-369)
        self.coefy = 0.1/(237-90)
        self.v_joint = np.arange(7)
        self.v_end = np.arange(6)
        self.cmd = {}
        self.found = False
        self.finish = 0
        self.distance = 10
        self.gripper = Gripper("right")
        #self.gripper.calibrate()

        cv2.namedWindow('image')
        self.np_image = np.zeros((400,640,3), np.uint8)
        cv2.imshow('image',self.np_image)
        #self._set_threshold()

        
        s = rospy.Service('vision_server_vertical', Vision, self.handle_vision_req)
        camera_topic = '/cameras/right_hand_camera/image'
        self.right_camera = rospy.Subscriber(camera_topic, Image, self._on_camera)
        ifr = rospy.Subscriber("/robot/range/right_hand_range/state", Range, self._read_distance)
        print "\nReady to use right hand vision server\n" 

        self.kin = baxter_kinematics('right')
        self.J = self.kin.jacobian()
        self.J_inv = self.kin.jacobian_pseudo_inverse()
        self.jointVelocity = np.asarray([1,2,3,4,5,6,7],np.float32)
        self.control_arm = Limb("right")
        self.control_joint_names = self.control_arm.joint_names()
        
        #print np.dot(self.J,self.jointVelocity)

    def _read_distance(self,data):

        self.distance = data.range



    def _set_threshold(self):
        
        cv2.createTrackbar('Min R(H)','image',0,255,nothing)
        cv2.createTrackbar('Max R(H)','image',255,255,nothing)
        cv2.createTrackbar('Min G(S)','image',0,255,nothing)
        cv2.createTrackbar('Max G(S)','image',255,255,nothing)
        cv2.createTrackbar('Min B(V)','image',0,255,nothing)
        cv2.createTrackbar('Max B(V)','image',255,255,nothing)

    def get_joint_velocity(self):
        cmd = {}
        velocity_list = np.asarray([1,2,3,4,5,6,7],np.float32)
        for idx, name in enumerate(self.control_joint_names):
            v = self.control_arm.joint_velocity( self.control_joint_names[idx])
            velocity_list[idx] = v
            cmd[name] = v 
        return velocity_list

    def list_to_dic(self,ls):
        cmd = {}
        for idx, name in enumerate(self.control_joint_names):
            v = ls.item(idx)
            cmd[name] = v 
        return cmd

    def PID(self):

        Kp = 0.5
        vy = -Kp*self.dx
        vx = -Kp*self.dy
        return vx,vy

    def _on_camera(self, data):

        self.busy = True
        self.framenumber += 1
        index = self.framenumber % 10
        cv2.namedWindow('image')
        cv_image = cv_bridge.CvBridge().imgmsg_to_cv(data, "bgr8")
        np_image = np.asarray(cv_image)
        image_after_process, mask = self.image_process(np_image)
        self.history_x[index] = self.dx*100
        self.history_y[index] = self.dy*100
        self.avg_dx = np.average(self.history_x)/100
        self.avg_dy = np.average(self.history_y)/100

        # if abs(self.dx)<0.01 and abs(self.dy)<0.01:
        #     self.found = True
        # else:
        #     self.found = False


        
        vx, vy = self.PID()
        self.v_end = np.asarray([(1-((abs(vx)+abs(vy))*5))*0.03,vy,vx,0,0,0],np.float32)
        #self.v_end = np.asarray([0,0,0,0,0,0.05],np.float32)
        self.v_joint = np.dot(self.J_inv,self.v_end)
        self.cmd = self.list_to_dic(self.v_joint)
              
        

        self.busy = False

        cv2.imshow('image',image_after_process)
        
        cv2.waitKey(1)


    def image_process(self,img):

        # min_r = cv2.getTrackbarPos('Min R(H)','image')
        # max_r = cv2.getTrackbarPos('Max R(H)','image')
        # min_g = cv2.getTrackbarPos('Min G(S)','image')
        # max_g = cv2.getTrackbarPos('Max G(S)','image')
        # min_b = cv2.getTrackbarPos('Min B(V)','image')
        # max_b = cv2.getTrackbarPos('Max B(V)','image')

        # min_r = 0
        # max_r = 57
        # min_g = 87  
        # max_g = 181
        # min_b = 37
        # max_b = 70

        min_r = 0
        max_r = 58
        min_g = 86  
        max_g = 255
        min_b = 0
        max_b = 255


        min_color = (min_r, min_g, min_b)
        max_color = (max_r, max_g, max_b)

        #img = cv2.flip(img, flipCode = -1)
        hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, min_color, max_color)
        

        result = cv2.bitwise_and(img, img, mask = mask)
        mask_bool = np.asarray(mask, bool)
        label_img = morphology.remove_small_objects(mask_bool, 1000, connectivity = 1)
        objects = measure.regionprops(label_img)

        if objects != []:
            self.found = True
            target = objects[0]
            box = target.bbox
            cv2.rectangle(img,(box[1],box[0]),(box[3],box[2]),(0,255,0),3)
            
            dx_pixel=target.centroid[1]-self.centerx
            dy_pixel=target.centroid[0]-self.centery
            
            dx=dx_pixel*self.coefx
            dy=dy_pixel*self.coefy

            angle = target.orientation           
            cv2.circle(img,(int(target.centroid[1]),int(target.centroid[0])),10,(0,0,255),-1)
                       
            self.dx = dx
            self.dy = dy
            #print self.dx,self.dy

        else:
            self.found = False
            self.dx = 0
            self.dy = 0


        return img, mask_bool

    def handle_vision_req(self, req):
        #resp = VisionResponse(req.requestID, 0, self.avg_dx, self.avg_dy)
        self.finish = 0
        if self.found == True:
            if self.distance > 0.07:
                self.control_arm.set_joint_velocities(self.cmd) 
                #rospy.sleep(0.02)
            else:
                self.finish = 1


        resp = VisionResponse(req.requestID, self.finish, self.dx, self.dy)

        
        return resp

    def clean_shutdown(self):
        print "Server finished"
        cv2.imwrite('box_img.png', self.blank_image)
        rospy.signal_shutdown("Done")
        sys.exit()
            
def test():
    position_list = [0.6,-0.3,0.3,3.14,0,-3.14]
    q = tf.transformations.quaternion_from_euler(position_list[3],position_list[4],position_list[5])
    print q
    euler = tf.transformations.euler_from_quaternion(q)
    print euler

if __name__=='__main__':
    
    vision = vision_server()
    
    done = False
    #arm = Limb("right")
    
    #print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        # if vision.found == False:
        #     vision.control_arm.set_joint_velocities(vision.cmd) 
        #     rate.sleep()
           
        
        # ep_position = arm.endpoint_pose()['position']
        # ep_orientation = arm.endpoint_pose()['orientation']
        # vision.control_arm.set_joint_velocities(vision.cmd)
        # euler = tf.transformations.euler_from_quaternion([ep_orientation.x,ep_orientation.y,ep_orientation.z,ep_orientation.w])
        # #print euler 
        # rospy.sleep(0.01)

        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                print ("Server finished")    
            else:
                print("key bindings: ")
                print("  Esc: Quit")

#!/usr/bin/env python
import sys
import rospy

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from baxter_interface import CameraController, Gripper, Limb
import numpy as np
import math
import copy


from baxter_pykdl import baxter_kinematics


                
def set_joints( target_angles_right = None, target_angles_left = None,timeout= 40000):

    right = Limb("right")
    left = Limb("left")
    
    if target_angles_right == None:
        reach_right = True
    else:
        reach_right = False
    

    if target_angles_left == None:
        reach_left = True
    else:
        reach_left = False
    
    time = 0

    while not reach_right or not reach_left:

            if target_angles_right: right.set_joint_positions(target_angles_right)
            if target_angles_left: left.set_joint_positions(target_angles_left)
            current_angles_right = right.joint_angles()
            current_angles_left = left.joint_angles()

            
            if reach_right == False:
                for k, v in current_angles_right.iteritems():
                    if abs(target_angles_right[k] - v) > 0.01:
                        reach_right = False
                        break
                    reach_right = True

            if reach_left == False:
                for k, v in current_angles_left.iteritems():
                    if abs(target_angles_left[k] - v) > 0.01:
                        reach_left = False
                        break
                    reach_left = True

            time+=1
            if time > timeout:
                print "Time out"
                break



def get_current_pose(hdr,arm,initial_pose = None):
    
    ep_position = arm.endpoint_pose()['position']
    ep_orientation = arm.endpoint_pose()['orientation']

    if initial_pose == None:
        
        current_pose = copy.deepcopy(initial_pose)
        current_pose = PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position=Point(
                            x=ep_position.x,
                            y=ep_position.y,
                            z=ep_position.z,
                        ),
                        orientation=Quaternion(
                            x=ep_orientation.x,
                            y=ep_orientation.y,
                            z=ep_orientation.z,
                            w=ep_orientation.w,
                        ),
                    )
        )
    else:
        current_pose = copy.deepcopy(initial_pose)
        current_pose.pose.position.x = ep_position.x
        current_pose.pose.position.y = ep_position.y
        current_pose.pose.position.z = ep_position.z

    return current_pose

def update_current_pose(current_pose,dx,dy,dz):
    new_pose = copy.deepcopy(current_pose)
    dx = dx/1.0
    dy = dy/1.0
    dz = dz/1.0
    new_pose.pose.position.x += dx
    new_pose.pose.position.y += dy
    new_pose.pose.position.z += dz
    #print dx,dy,dz
    return new_pose

def ik(pose):
    ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq.pose_stamp.append(pose)

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    if (resp.isValid[0]):
        #print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        arm = Limb("right")
        arm.set_joint_positions(limb_joints)
        return True
        #rospy.sleep(0.05)

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return False

def ik_pykdl(arm, kin, pose, arm_position = 'right'):
    position = pose.pose.position
    orientation = pose.pose.orientation
    pos = [position.x,position.y,position.z]
    rot = [orientation.x,orientation.y,orientation.z,orientation.w]
    joint_angles = kin.inverse_kinematics(pos,rot)
    if joint_angles:
        if arm_position == 'right':
            cmd = {
                'right_s0': joint_angles[0],
                'right_s1': joint_angles[1],
                'right_e0': joint_angles[2],
                'right_e1': joint_angles[3],
                'right_w0': joint_angles[4],
                'right_w1': joint_angles[5],
                'right_w2': joint_angles[6],
            }
        else:
            cmd = {
                'left_s0': joint_angles[0],
                'left_s1': joint_angles[1],
                'left_e0': joint_angles[2],
                'left_e1': joint_angles[3],
                'left_w0': joint_angles[4],
                'left_w1': joint_angles[5],
                'left_w2': joint_angles[6],
            }

        arm.set_joint_positions(cmd)
        return True
    else:
        return False

def ik_move_to_pose(arm,kin,pose,timeout= 60000):
    position = pose.pose.position
    orientation = pose.pose.orientation
    pos = [position.x,position.y,position.z]
    rot = [orientation.x,orientation.y,orientation.z,orientation.w]
    joint_angles = kin.inverse_kinematics(pos,rot)
    if joint_angles:
        cmd = {
            'right_s0': joint_angles[0],
            'right_s1': joint_angles[1],
            'right_e0': joint_angles[2],
            'right_e1': joint_angles[3],
            'right_w0': joint_angles[4],
            'right_w1': joint_angles[5],
            'right_w2': joint_angles[6],
        }

        set_joints(target_angles_right = cmd,timeout = timeout)
    
def ik_move(hdr, arm, kin, arm_position = 'right', target_dx = None, target_dy = None, target_dz = None, x = None, y = None, z = None, timeout= 50000,speed = 1.0, speedx = 1.0,speedy = 1.0,speedz = 1.0):

    initial_pose = get_current_pose(hdr,arm)
    target_x = initial_pose.pose.position.x
    target_y = initial_pose.pose.position.y
    target_z = initial_pose.pose.position.z
    
    if target_dx != None: target_x += target_dx
    if target_dy != None: target_y += target_dy
    if target_dz != None: target_z += target_dz

    if x != None: target_x = x
    if y != None: target_y = y
    if z != None: target_z = z

    dx = 100
    dy = 100
    dz = 100

    solution_found = 1

    time = 0

    while (abs(dx) > 0.01 or abs(dy) > 0.01 or abs(dz) > 0.01) and solution_found == 1:
    
    #while (abs(dx) > 0.01 or abs(dy) > 0.01 or abs(dz) > 0.01) and i < 5000:
        
        current_pose = get_current_pose(hdr,arm,initial_pose)
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_z = current_pose.pose.position.z
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z

        pdx = dx*speed*speedx
        pdy = dy*speed*speedy
        pdz = dz*speed*speedz
        #print dx, dy, dz
        new_pose = update_current_pose(current_pose,pdx,pdy,pdz)
        solution_found = ik_pykdl(arm,kin,new_pose, arm_position)
        rospy.sleep(0.05)
        time += 1
        if time > timeout:
            print "Time out"
            break

    return solution_found
        
def ik_move_one_step(initial_pose, predict_pose, hdr, arm, kin, target_dx = None, target_dy = None, target_dz = None):



    current_pose = get_current_pose(hdr,arm,initial_pose)

    if target_dx == None: 
        target_dx = initial_pose.pose.position.x - current_pose.pose.position.x
    else: 
        target_dx = target_dx + predict_pose.pose.position.x - current_pose.pose.position.x
    if target_dy == None: 
        target_dy = initial_pose.pose.position.y - current_pose.pose.position.y
    else: 
        target_dy = target_dy + predict_pose.pose.position.y - current_pose.pose.position.y
    if target_dz == None: 
        target_dz = initial_pose.pose.position.z - current_pose.pose.position.z
    else:
        target_dz = target_dz + predict_pose.pose.position.z - current_pose.pose.position.z


    new_pose = update_current_pose(current_pose,target_dx,target_dy,target_dz)
    solution_found = ik_pykdl(arm, kin, new_pose)
    return solution_found, new_pose



def main():

    joint_states = {
        
        'right_initial':{
            'right_e0': 2.498,
            'right_e1': 2.158,
            'right_s0': 0.826, 
            'right_s1': 0.366,
            'right_w0': 2.809,
            'right_w1': 1.867,
            'right_w2': 0.411,
        },
        'left_initial':{
            'left_e0': -1.738,
            'left_e1': 1.828,
            'left_s0': 0.247, 
            'left_s1': 0.257,
            'left_w0': 0.0721,
            'left_w1': -0.818,
            'left_w2': 1.826,
        },
        
    }



    rospy.init_node('demo', anonymous=True)
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    arm_right = Limb("right")
    kin_right = baxter_kinematics('right')
    arm_left = Limb("left")
    kin_left = baxter_kinematics('left')
    
    set_joints(target_angles_right = joint_states['right_initial'])
    # right arm movement
    ik_move(hdr,arm_right, kin_right, target_dx = 0.35, arm_position = 'right', speedx = 0.05)
    ik_move(hdr,arm_right, kin_right, target_dy = 0.1, arm_position = 'right', speedy = 0.1)
    ik_move(hdr,arm_right, kin_right, target_dx = -0.35, arm_position = 'right', speedx = 0.1)
    
     
if __name__=='__main__':
    sys.exit(main())
    
#!/usr/bin/env python

import herbpy
import os
import numpy as np
import math
import openravepy
import sys 
import prpy

from prpy.perception.apriltags import ApriltagsModule
from prpy.util import FindCatkinResource

from catkin.find_in_workspaces import find_in_workspaces
package_name = 'pr_ordata'
directory = 'data/objects'
objects_path = find_in_workspaces(
	search_dirs=['share'],
	project=package_name,
	path=directory,
	first_match_only=True)
if len(objects_path) == 0:
	print('Can\'t find directory %s %s' % (package_name, directory))
	sys.exit()
else:
	print objects_path
	objects_path = objects_path[0]

env, robot = herbpy.initialize(sim=False, attach_viewer='rviz', segway_sim=True)

#Add the table

ap_detect = ApriltagsModule(marker_topic='/apriltags_kinect2/marker_array',
                              marker_data_path=FindCatkinResource('pr_ordata',
                                                                  'data/objects/tag_data.json'),
                              kinbody_path=FindCatkinResource('pr_ordata',
                                                              'data/objects'),
                              destination_frame='/herb_base',
                              detection_frame='/head/kinect2_rgb_optical_frame',			      
                              reference_link=robot.GetLink('/herb_base'))

objs = ap_detect.DetectObjects(robot)
table = objs[0]
# add a cube to the world to the table
cube_path = os.path.join(objects_path, 'rubis_cube.kinbody.xml')
cube = env.ReadKinBodyXMLFile(cube_path)
if cube == None:
	print('Can\'t load the cube kinbody')
	sys.exit()
table_aabb= table.ComputeAABB()
x = table_aabb.pos()[0] + table_aabb.extents()[0]*0
y = table_aabb.pos()[1] + table_aabb.extents()[1]*0.6
table_center = table_aabb.pos()[:2]
z = table_aabb.pos()[2] + table_aabb.extents()[2]+0.01

cube_pose = cube.GetTransform()
cube_pose[:3,3] =np.transpose([x,y,z])
cube_pose[:2,3] =table_center
cube_pose[2,3] = z
cube_pose[1,3] = cube_pose[1,3] + 0.3
cube.SetTransform(cube_pose)
env.AddKinBody(cube)

def rotation_x(theta):
	rot = np.array([[1,0,0,0], [0, math.cos(theta), -math.sin(theta), 0],[0, math.sin(theta), math.cos(theta), 0],[0,0,0,1]])
	return rot

def rotation_y(theta):
	rot = np.array([[math.cos(theta),0,math.sin(theta),0], [0, 1, 0, 0],[-math.sin(theta), 0, math.cos(theta), 0],[0,0,0,1]])
	return rot

def rotation_z(theta):
	rot = np.array([[math.cos(theta), -math.sin(theta),0,0], [math.sin(theta), math.cos(theta), 0, 0],[0, 0, 1, 0],[0,0,0,1]])
	return rot

def grab_cube(robot, cube, push):
	try:
		manip = robot.GetActiveManipulator()
		manip.PlanToTSR(herbpy.tsr.rubiscube_grasp(robot, cube, push, manip), smoothingitrs=100, execute=True)
		manip.hand.CloseHand()
		manip.GetRobot().Grab(cube)
	except Exception, e:
		print 'Grasping the cube failed: ', str(e)

def lift_cube(robot, cube, dist):
	try:
		manip = robot.GetActiveManipulator()
		manip.PlanToTSR(herbpy.tsr.poptarts_lift(robot, cube, manip, dist), smoothingitrs=100, execute=True)
	except Exception, e:
		print 'lifting cube failed: ', str(e)	

def cubetsr(robot, rubis_cube, r1 = 0, r2 = 0, p1 = 0, p2 = 0, y1 = 0, y2 = 0, push_distance = 0.0, manip = None):
    """
    @param robot The robot performing the grasp
    @param pop_tarts The pop tarts box to grasp
    @param push_distance The distance to push before grasping
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    T0_w = rubis_cube.GetTransform()
    ee_to_palm_distance = 0.18
    default_offset_distance = 0.04 # This is the radius of the box
                                   # plus a little bit
    total_offset = ee_to_palm_distance + default_offset_distance + push_distance
    Tw_e = np.array([[ 0., 0., 1., -total_offset], 
                        [1., 0., 0., 0.], 
                        [0., 1., 0., 0.06], # half of box height
                        [0., 0., 0., 1.]])

    Bw = np.zeros((6,2))
    Bw[0,:] = [-0.07, -0.06]
    Bw[1,:] = [-0.01, -0.01]
    Bw[2,:] = [0.02, 0.04]  # Allow a little vertical movement
    Bw[3,:] = [r1, r2]
    Bw[4,:] = [p1, p2]  # Allow any orientation
    Bw[5,:] = [y1, y2]
    grasp_tsr = prpy.tsr.TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = prpy.tsr.TSRChain(sample_start=False, sample_goal = True, 
            constrain=False, TSR = grasp_tsr)

    return [grasp_chain]

def event_seq():
	# Grabbing the cube
	chain = cubetsr(robot, cube, y1=-0.1, y2=0.1, p1=1.4, p2=1.6, push_distance=0.05)
	robot.PushGrasp(cube, tsrlis=chain, push_distance=0.05)
	
	# Move it into midair
	direction=[0,0,1]
	distance=0.1
	robot.left_arm.PlanToEndEffectorOffset(direction, distance, execute=True)
	rotx = rotation_x(np.pi/2)
	current_pose = robot.left_arm.GetEndEffectorTransform()
	next_pose = np.dot(current_pose, rotx)
	robot.left_arm.PlanToEndEffectorPose(next_pose, execute=True)

# the viewer stays open after the example is done
import IPython; IPython.embed()


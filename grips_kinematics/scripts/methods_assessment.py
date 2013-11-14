#!/usr/bin/env python
import roslib; roslib.load_manifest('grips_kinematics')
import rospy, math, argparse
import random
import scipy.io as sio
import numpy as np
# Services
from grips_msgs.srv import GetStateMetrics, GetStateMetricsRequest, GetStateMetricsResponse
from grips_msgs.srv import GetJointLimits, GetJointLimitsRequest, GetJointLimitsResponse
from grips_msgs.srv import GetPoseMetrics, GetPoseMetricsRequest, GetPoseMetricsResponse
# Messages
from geometry_msgs.msg import Pose

q_goal_str = 'desired_angles_mat'
poses_goal_str = 'desired_poses_mat'

q_calc_str = 'calculated_angles_mat'
iter_str = 'iterations'

joint_names = ['SA', 'SE', 'linkage_tr', 'WP', 'WY', 'WR']
NUM_JOINTS = len(joint_names)

def generate_input_data(input_mat_file, NUM_TEST = 100):
	# Subscribe to the FK service
	fk_srv_name = rospy.get_param('metrics_service', '/grips/kinematic_srv/get_fk_metrics')
	rospy.loginfo('Waiting for %s service' % fk_srv_name)
	rospy.wait_for_service(fk_srv_name)
	fk_srv = rospy.ServiceProxy(fk_srv_name, GetStateMetrics)
  # Subscribe to limits_service
	limits_srv_name = rospy.get_param('limits_service', '/grips/kinematic_srv/get_joint_limits')
	rospy.loginfo('Waiting for %s service' % limits_srv_name)
	rospy.wait_for_service(limits_srv_name)
	limits_srv = rospy.ServiceProxy(limits_srv_name, GetJointLimits)
	# Get the joint limits
	req = GetJointLimitsRequest()
	req.header.stamp = rospy.Time.now()
	req.header.frame_id = '/world'
	req.name = joint_names
	try:
		res = limits_srv(req)
	except rospy.ServiceException, e:
		rospy.logwarn('Service did not process request: %s' % str(e))			
	min_positions = list(res.min_position)
	max_positions = list(res.max_position)
	min_positions[-1] = -2*math.pi; max_positions[-1] = 2*math.pi;
	#~ min_positions = [-1.37, 	-1.37,  -0.5236, 	-0.4, 	-0.42, 		-6.283]
	#~ max_positions = [1.37,		0, 			0.3, 			0.61, 	0.42, 		6.283]
	rospy.loginfo('min_positions: %s' % str(min_positions))
	rospy.loginfo('max_positions: %s' % str(max_positions))
	rospy.loginfo('Generating test MAT file, [%d] tests' % NUM_TEST)
	angles_mat = np.array([]) 
	poses_mat = np.array([])
	req = GetStateMetricsRequest()
	for i in xrange(NUM_TEST):
		random_joints = []
		current_pose = [0]*7
		for joint in xrange(NUM_JOINTS):
			random_joints.append(random.uniform(min_positions[joint], max_positions[joint]))
		req.joint_states.header.stamp = rospy.Time.now()
		req.joint_states.header.frame_id = '/world'
		req.joint_states.name = joint_names
		req.joint_states.position = list(random_joints)
		try:
			res = fk_srv(req)
			if res.found_group:
				current_pose = pose2list(res.pose)
		except rospy.ServiceException, e:
			rospy.logwarn('Service did not process request: %s' % str(e))
		angles_mat = np.append(angles_mat, np.array(random_joints), 0)
		poses_mat = np.append(poses_mat, np.array(current_pose), 0)
		if (i % 1000 == 0):
			rospy.loginfo('[FK] Evaluated: %d/%d' % (i, NUM_TEST))
		# Check for shutdowns
		if rospy.is_shutdown():
			return
	# Prepares the data to save it in a .mat file
	angles_shape = (NUM_TEST, NUM_JOINTS)
	poses_shape = (NUM_TEST, 7)
	angles_mat = angles_mat.reshape(angles_shape)
	poses_mat = poses_mat.reshape(poses_shape)
	rospy.loginfo('[FK] Evaluated: %d/%d' % (i+1, NUM_TEST))
	rospy.loginfo('Writing %s file' % input_mat_file)
	sio.savemat(input_mat_file, {q_goal_str:angles_mat, poses_goal_str:poses_mat}, oned_as='column')
	rospy.sleep(3.0)

def solve_ik(input_mat_file, output_mat_file):	
	# Subscribe to kinematic services
	ik_srv_name = rospy.get_param('metrics_service', '/grips/kinematic_srv/get_ik_metrics')
	rospy.loginfo('Waiting for %s service' % ik_srv_name)
	rospy.wait_for_service(ik_srv_name)
	ik_srv = rospy.ServiceProxy(ik_srv_name, GetPoseMetrics)
  # Load data from the *.mat file  
	input_dict = sio.loadmat(input_mat_file)
	poses_mat = input_dict[poses_goal_str]
	calculated_mat = np.array([])  
	iterations = []
	calculated = 0	
	req = GetPoseMetricsRequest()
	tests = poses_mat.shape[0]
	for i, pose in enumerate(poses_mat):
		req.header.stamp = rospy.Time.now()
		req.header.frame_id = '/world'
		req.link_name = 'end_effector'
		req.pose = list2pose(pose)
		try:
			res = ik_srv(req)
			if res.found_ik and res.found_group:
				calculated_mat = np.append(calculated_mat, np.array(res.joint_states.position), 0)
				iterations.append(1)
				calculated += 1
			else:
				calculated_mat = np.append(calculated_mat, np.array([0]*NUM_JOINTS), 0)
				iterations.append(0)
		except rospy.ServiceException, e:
			rospy.logwarn('Service did not process request: %s' % str(e))
		if (i % 1000 == 0):
			rospy.loginfo('[IK] Evaluated: %d/%d' % (i, tests))
		# Check for shutdowns
		if rospy.is_shutdown():
			return
	# Show the result in the console
	rospy.loginfo('IK Done: %d/%d metrics were calculated' % (calculated, tests))
	#~ rospy.loginfo('Iterations len: %d' % (len(iterations)))
	#~ print calculated_mat.shape
	# Prepare the data to save it in a .mat file
	mat_shape = (tests, NUM_JOINTS)
	calculated_mat = calculated_mat.reshape(mat_shape)
	data_dict = {q_calc_str: calculated_mat, iter_str:iterations}
	rospy.loginfo('Writing %s file' % output_mat_file)
	sio.savemat(output_mat_file, data_dict, oned_as='column')

def pose2list(pose):
	data = [0] * 7
	data[0] = pose.orientation.w
	data[1] = pose.orientation.x
	data[2] = pose.orientation.y
	data[3] = pose.orientation.z
	data[4] = pose.position.x
	data[5] = pose.position.y
	data[6] = pose.position.z
	return data

def list2pose(data):
	pose = Pose()
	pose.orientation.w = data[0]
	pose.orientation.x = data[1]
	pose.orientation.y = data[2]
	pose.orientation.z = data[3]
	pose.position.x = data[4]
	pose.position.y = data[5]
	pose.position.z = data[6]
	return pose


if __name__ == '__main__':
	rospy.init_node('methods_assessment')
	# Parse the arguments
	parser = argparse.ArgumentParser(description='Performs the assessment of the current IK solver')
	parser.add_argument('--generate', action='store_true', 
												help='If set, generates a new input MAT file')
	parser.add_argument('--input', dest='input_file', type=str, required=True,
												help='The path of the input MAT file')
	parser.add_argument('--output', dest='output_file', type=str, required=True,
												help='The path of the output MAT file')
	parser.add_argument('--tests', dest='tests', type=int, default=1000,
												help='Tests to perform')
	args = parser.parse_args()
	#Generate a new input data file
	if args.generate:
		generate_input_data(args.input_file, args.tests)
	# Test the IK solver
	solve_ik(args.input_file, args.output_file)
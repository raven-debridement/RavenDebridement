#!/usr/bin/env python

import roslib
roslib.load_manifest('RavenDebridement')
roslib.load_manifest('tfx')
import rospy
import sys

from raven_2_msgs.msg import *
from tfx.canonical import *

import optparse
from collections import defaultdict
import os, math

import pylab
import numpy as np

from threading import Lock

# example joint err:
#              SHOULD  ELBOW   INSERT   ROTAT   PITCH   FIN1    FIN2    YAW     GRASP
# joint_avg_:  -0.034  -1.104   0.052   2.230   1.678   1.661   1.521  -0.070   3.164
# joint_avg_:  -0.285  -0.500   0.024   1.553   1.010   0.967   1.256   0.144   2.205
# joint_avg_:  -0.159  -0.913   0.016   2.086   1.377   1.290   1.270  -0.010   2.513
# joint_avg_:  -0.127  -1.144   0.050   2.436   1.600   1.413   1.336  -0.039   2.699
# joint_avg_:  -0.219  -1.309   0.042   2.864   2.034   1.990   1.704  -0.143   3.644
# joint_avg_:  -0.108  -1.169  -0.000   2.476   1.720   1.333   1.511   0.089   2.794

# average of the above joint errors:
# joint_avg_:  -0.155  -1.023   0.031   2.274   1.570   1.442   1.433   0.005   2.836


#Runlevel: PEDAL_UP
#Main loop number: 380867
#Master mode: NONE
#Controller: joint_torque_control
#Green arm:	Board 0, type 23:
#type:		SHOULD	ELBOW_	Z_INS_	TOOL_R	WRIST_	GRASP1	GRASP2	
#enc_val:	40795	-2930	57	153	279	363	92	
#enc_off:	-705	-118839	63889	21456	21420	25568	25614	
#mpos:		65.19	182.07	-100.27	-33.46	-33.21	-39.59	-40.09	
#mpos_d:		65.24	181.56	-100.27	-33.47	-33.21	-39.59	-40.09	
#mvel:		0	-0	-0	0	-0	-0	0	
#mvel_d:		0	0	0	0	0	0	0	
#jpos:		0.52	1.62	-0.05	0.16	0.12	-0.59	-0.64	
#jpos_d:		0.52	1.61	-0.05	0.16	0.12	-0.59	-0.64	
#jvel:		0.00	-0.00	-0.00	0.00	0.00	0.00	0.00	
#jvel_d:		0.00	-0.00	-0.00	0.00	0.00	0.00	0.00	
#tau_d:		0.000	0.000	0.000	0.000	0.000	0.000	0.000	
#DAC:		0	0	0	0	0	0	0	
#KP gains:	0.060	0.050	0.020	0.090	0.050	0.020	0.020	
#KD gains:	0.003	0.004	0.001	0.001	0.000	0.000	0.000	
#KI gains:	0.000	0.000	0.000	0.050	0.050	0.020	0.020	


class StatePrinter(object):
	
	
	def short_name(self,name):
		if self.short_names.has_key(name):
			return self.short_names[name]
		else:
			return name
	
	def __init__(self,options,enabled):
		self.options = options
		self.enabled = enabled
		
		if options.degrees:
			self.conv = 180. / math.pi
		else:
			self.conv = 1.
		
		self.duration = rospy.Duration(1./options.rate)
		self.last_print_time = None
		
		self.output = None
		
		self.joints_enabled = options.joints
		if self.joints_enabled and enabled and set(['set_points','state']).isdisjoint(enabled):
			for name in enabled:
				if name.startswith('j'):
					break
			else:
				self.joints_enabled = False
		
		self.motors_enabled = options.joints
		if self.motors_enabled and enabled and set(['set_points','torque']).isdisjoint(enabled):
			for name in enabled:
				if name.startswith('m'):
					break
			else:
				self.motors_enabled = False
		
		self.short_names = {
				'joint_states': 'state',
				'joint_positions': 'jpos',
				'joint_velocities': 'jvel',
				'joint_command_types': 'jcmd_type',
				'joint_command_values': 'jcmd',
				'joint_set_point_positions': 'jpos_d',
				'joint_set_point_velocities': 'jvel_d',
				
				'motor_positions': 'mpos',
				'motor_velocities': 'mvel',
				'motor_torques': 'torque',
				
				'motor_set_point_positions': 'mpos_d',
				'motor_set_point_velocities': 'mvel_d',
				
				'motor_enc_vals': 'enc_value',
				'motor_enc_offsets': 'enc_offset',
				'motor_dac_cmds': 'dac_cmd'
				}
		
		self.default_off = set(['motor_enc_vals','motor_enc_offsets','motor_dac_cmds'])
		self.nick_names = defaultdict(list,**{
						'jcmd':['joint_command_types'],
						'jstate':['joint_states'],
						'joints':['jpos','jvel'],
						'set_points':['jpos_d','jvel_d',
									'mpos_d','mvel_d'],
						'pose':['tool_pose'],
						'tool':['tool_pose','grasp']
						})
                
                # ADDED
                # True if last print_state had a pedal down
                self.wasLastPedalDown = True
                self.accumTimes = 0
		self.justStoppedAccum = False
	
                self.fields = {}

		#self.jointErrorLock = Lock()

		#self.figure = pylab.figure('Joint Error Graph')
		#self.graph = self.figure.add_subplot(111)

		#pylab.show(block=False)

	def is_enabled(self,field):
		if not self.enabled:
			return field not in self.default_off
		elif field in self.enabled:
			return True
		elif self.short_name(field) in self.enabled:
			return True
		else:
			for name in self.enabled:
				if field in self.nick_names[name] or self.short_name(field) in self.nick_names[name]:
					return True
			return False
	
	def add_line(self,line,*args):
		if not all(map(self.is_enabled,args)):
			return
		self.output += line.format(**(self.fields)) + '\n'
	
	def state_callback(self,msg):
		now = msg.header.stamp
		if self.last_print_time and now - self.last_print_time < self.duration:
			return
		self.last_print_time = now
		
		conv = self.conv
		
                # ADDED
		#self.fields = {}
		
		self.output = ''
		add_line = lambda line: self.output + line.format(**self.fields) + '\n'
		
		#std_msgs/Header header
		#  uint32 seq
		self.fields['loop_number'] = msg.header.seq
		#  time stamp
		self.fields['stamp'] = msg.header.stamp.to_sec()
		#  string frame_id
		self.fields['frame_id'] = msg.header.frame_id
		#uint8 runlevel
		#uint8 sublevel
		runlevel_strs = defaultdict(lambda: 'UNKNOWN',{0:'E_STOP',1:'INIT',2:'PEDAL_UP',3:'PEDAL_DOWN'})
		init_sublevel_strs = defaultdict(lambda: 'UNKNOWN',{0:'0',1:'1',2:'2',3:'3'})
		
		self.fields['runlevel'] = runlevel_strs[msg.runlevel]
		if msg.runlevel == 1:
			self.fields['runlevel'] += ':' + init_sublevel_strs[msg.sublevel]

		#bool pedal_down
		self.fields['pedal_down'] = msg.pedal_down
                
                
                # ADDED
                if not self.wasLastPedalDown and self.fields['pedal_down']:
                    pedalChanged = True
                    accumulate = True
                    self.accumTimes += 1
                elif self.wasLastPedalDown and self.fields['pedal_down']:
                    pedalChanged = False
                    accumulate = True
                    self.accumTimes += 1
		elif self.wasLastPedalDown and not self.fields['pedal_down']:
                    self.justStoppedAccum = True
		    pedalChanged = True
		    accumulate = False
		    self.accumTimes = 0
		else:
                    pedalChanged = False
                    accumulate = False
                    self.accumTimes = 0
                    
                self.wasLastPedalDown = self.fields['pedal_down']


		"""
		elif self.wasLastPedalDown and not self.fields['pedal_down']:
                    
		    if self.fields.has_key('joint_errs'):
                        #X = np.array(range(self.accumTimes))
                        JE0_list = self.fields['joint_errs'][0]
			JE0 = np.array(JE0_list)
			X = np.array(range(len(JE0_list)))

			#rospy.loginfo(len(JE0_list))
			if len(JE0_list) > 0:
                            #figure = pylab.figure('Joint Error Graph')
			    #graph = fig.add_subplot(111)
			    #graph.clear()
                            self.graph.clear()
                            self.graph.plot(X,JE0)
			    #pylab.show(block=False)

                            #pylab.cla()
			    #graph = fig.add_subplot(111)
			    #graph.plot(X, JE0)
			    #fig.show()
			    #fig.canvas.draw()
		"""

		#string master
		#uint8 controller
		self.fields['_master'] = msg.master
		self.fields['controller'] = Constants.CONTROLLER_STRINGS.split(',')[msg.controller]
		
#		self.output = add_line('Runlevel:   {runlevel:10.10s}')
#		self.output = add_line('Timestamp:  {stamp:f} [{loop_number:d}]')
#		self.output = add_line('Master:     {master}')
#		self.output = add_line('Controller: {controller}')
		self.add_line('Timestamp:  {stamp:f} [{loop_number:d}]')
		self.add_line('Frame:      {frame_id:s}')
		self.add_line('Runlevel:   {runlevel:10.10s}')
		self.add_line('Master:     {_master}')
		self.add_line('Controller: {controller}')
					
		#raven_2_msgs/ArmState[] arms
		for arm in msg.arms:
			if self.options.arm and arm.name not in self.options.arm:
				continue
			
			#  string name
			self.fields['arm_name'] = arm.name
			
			#  uint8 type
			self.fields['arm_type'] = Constants.ARM_TYPE_STRINGS.split(',')[arm.type]
			
			self.output = add_line('\n****Arm {arm_name} [{arm_type}]****')
			
			#  geometry_msgs/Pose base_pose
			base_pose = transform(arm.base_pose)
			self.fields['base_pose'] = base_pose
			
			#  uint8 tool_type
			self.fields['tool_type'] = Constants.TOOL_TYPE_STRINGS.split(',')[arm.tool_type]
			
			#  raven_2_msgs/ToolState tool
			#    geometry_msgs/Pose pose
			#    float32 grasp
			tool_pose = transform(arm.tool.pose)
			self.fields['tool_pose'] = tool_pose
			self.fields['grasp'] = arm.tool.grasp * conv
			
			tool_pose_desired = transform(arm.tool_set_point.pose)
			self.fields['tool_pose_desired'] = tool_pose_desired
			self.fields['grasp_desired'] = arm.tool_set_point.grasp * conv
			
#			self.output = add_line('Pose: ({tool_pose.position.x:.3f}, {tool_pose.position.y:.3f}, {tool_pose.position.z:.3f})')
#			self.output = add_line('      {tool_pose.tb_angles}')
#			self.output = add_line('Grasp: {grasp:.3f}')
			self.add_line('','tool_pose','grasp')
			self.add_line('Pose:         ({tool_pose.position.x:.3f}, {tool_pose.position.y:.3f}, {tool_pose.position.z:.3f})','tool_pose')
			self.add_line('              {tool_pose.tb_angles}','tool_pose')
			self.add_line('Pose command: ({tool_pose_desired.position.x:.3f}, {tool_pose_desired.position.y:.3f}, {tool_pose_desired.position.z:.3f})','tool_pose_desired')
			self.add_line('              {tool_pose_desired.tb_angles}','tool_pose_desired')
			self.add_line('Grasp:         {grasp:.3f}','grasp')
			self.add_line('Grasp command: {grasp:.3f}','grasp_desired')
			
			#  raven_2_msgs/ToolSetPoint tool_set_point
			#    geometry_msgs/Pose pose
			#    float32 grasp
			tool_pose_desired = transform(arm.tool_set_point.pose)
			self.fields['tool_pose_desired'] = tool_pose_desired
			self.fields['grasp_desired'] = arm.tool_set_point.grasp
			
			self.fields['joint_types'] = []
			self.fields['joint_states'] = []
			self.fields['joint_positions'] = []
			self.fields['joint_velocities'] = []
			self.fields['joint_command_types'] = []
			self.fields['joint_command_values'] = []
			self.fields['joint_set_point_positions'] = []
			self.fields['joint_set_point_velocities'] = []
			
			self.fields['motor_types'] = []
			self.fields['motor_positions'] = []
			self.fields['motor_velocities'] = []
			self.fields['motor_torques'] = []
			
			self.fields['motor_set_point_positions'] = []
			self.fields['motor_set_point_velocities'] = []
			
			self.fields['motor_enc_vals'] = []
			self.fields['motor_enc_offsets'] = []
			self.fields['motor_dac_cmds'] = []

                        # ADDED
                        # two purposes: initialize if first time
                        # set accumulated errors to zero if pedal just changed to down
                        if (not self.fields.has_key('joint_avg_errs')) or (pedalChanged and accumulate):
                            self.fields['joint_avg_errs'] = [0 for joint in arm.joints]
                        if (not self.fields.has_key('motor_avg_errs')) or (pedalChanged and accumulate):
                            self.fields['motor_avg_errs'] = [0 for _ in range(len(arm.joints)-2)]
			if (not self.fields.has_key('joint_errs')) or (pedalChanged and accumulate):
                            self.fields['joint_errs'] = [list() for joint in arm.joints]
                        if (not self.fields.has_key('motor_errs')) or (pedalChanged and accumulate):
                            self.fields['motor_errs'] = [list() for _ in range(len(arm.joints)-2)]

			
			#  raven_2_msgs/JointState[] joints
			for joint in arm.joints:
				if joint.type == Constants.JOINT_TYPE_INSERTION:
					conv = 100.
				
				#    uint16 type
				self.fields['joint_types'].append(Constants.JOINT_TYPE_STRINGS.split(',')[joint.type])
				#    int16 state
				self.fields['joint_states'].append(JointState.STATE_STRINGS.split(',')[joint.state])
				
				#    float32 position
				self.fields['joint_positions'].append(joint.position * conv)
				#    float32 velocity
				self.fields['joint_velocities'].append(joint.velocity * conv)
				
				
				#    raven_2_msgs/JointCommand command
				#      uint8 command_type
				#      float32 value
				self.fields['joint_command_types'].append(JointCommand.COMMAND_TYPE_STRINGS.split(',')[joint.command.command_type])
				if joint.command.command_type == JointCommand.COMMAND_TYPE_TORQUE:
					self.fields['joint_command_values'].append(joint.command.value)
				else:
					self.fields['joint_command_values'].append(joint.command.value * conv)
				#    raven_2_msgs/DOFSetPoint set_point
				#      float32 position
				#      float32 velocity
				self.fields['joint_set_point_positions'].append(joint.set_point.position * conv)
				self.fields['joint_set_point_velocities'].append(joint.set_point.velocity * conv)

				
				if joint.type not in (Constants.JOINT_TYPE_GRASP,Constants.JOINT_TYPE_YAW):
					self.fields['motor_types'].append(Constants.JOINT_TYPE_STRINGS.split(',')[joint.type])
					
					#    float32 motor_position
					self.fields['motor_positions'].append(joint.motor_position * conv)
					#    float32 motor_velocity
					self.fields['motor_velocities'].append(joint.motor_velocity * conv)
					#    float32 torque
					self.fields['motor_torques'].append(joint.torque)
					
					#    float32 gravitation_torque_estimate
					#    float32 integrated_position_error
					
					#    raven_2_msgs/DOFSetPoint set_point
					#      float32 motor_position
					#      float32 motor_velocity
					self.fields['motor_set_point_positions'].append(joint.set_point.motor_position * conv)
					self.fields['motor_set_point_velocities'].append(joint.set_point.motor_velocity * conv)
					
					#    int32 encoder_value
					self.fields['motor_enc_vals'].append(joint.encoder_value)
					#    int32 encoder_offset
					self.fields['motor_enc_offsets'].append(joint.encoder_offset)
					#    int16 dac_command
					self.fields['motor_dac_cmds'].append(joint.dac_command)

                        # ADDED
                        if accumulate:
                            #self.jointErrorLock.acquire()
                            numJoints = len(self.fields['joint_avg_errs'])
                            for index in range(numJoints):
                                jpos = self.fields['joint_positions'][index]
                                jpos_d = self.fields['joint_set_point_positions'][index]
                                avg_err = self.fields['joint_avg_errs'][index]
                                total_err = avg_err*(self.accumTimes - 1)
                                self.fields['joint_avg_errs'][index] = (total_err + (jpos_d - jpos))/self.accumTimes

				self.fields['joint_errs'][index].append(jpos_d - jpos)
			    #self.jointErrorLock.release()

                            numMotors = len(self.fields['motor_avg_errs'])
                            for index in range(numMotors):
                                mpos = self.fields['motor_positions'][index]
                                mpos_d = self.fields['motor_set_point_positions'][index]
                                avg_err = self.fields['motor_avg_errs'][index]
                                total_err = avg_err*(self.accumTimes - 1)
                                # unsure why the following is off. but order of magnitude is correct
                                self.fields['motor_avg_errs'][index] = (total_err + (mpos_d - mpos))/self.accumTimes

				self.fields['motor_errs'][index].append(mpos_d - mpos)
			
			field_name_format = '{field_name:10.10}:'
			num_field_format = '{{{field_num:d}: 7.{precision:d}f}}'
			str_field_format = '{{{field_num:d}:>7.{precision:d}}}'
			
			header_field_name_format = '{field_name:10.10} '
			header_field_format = '{{{field_num:d}:^7.7}}'
			
			header_fmt = ' '.join([header_field_name_format] + ([header_field_format.format(field_num=i) for i in xrange(len(arm.joints))]))
			
			
			
			line_fmt = lambda field_format,precision: ' '.join([field_name_format] + ([field_format.format(field_num=i,precision=precision) for i in xrange(len(arm.joints))]))
			
			num_line_fmt = lambda precision: line_fmt(num_field_format,precision)
			str_line_fmt = lambda precision=7: line_fmt(str_field_format,precision)
			line = lambda field_name, precision: num_line_fmt(precision).format(*(self.fields[field_name]),field_name=self.short_name(field_name))
			str_line = lambda field_name,precision: str_line_fmt(precision).format(*(self.fields[field_name]),field_name=self.short_name(field_name))
			
			add_line = lambda field_name,precision: self.add_line(line(field_name,precision),field_name)
			add_str_line = lambda field_name,precision: self.add_line(str_line(field_name,precision),field_name)
			
			if self.joints_enabled:
				self.add_line('\n------------------------------JOINTS------------------------------')
				self.add_line(header_fmt.format(*(self.fields['joint_types']),field_name=''))
				add_str_line('joint_states',7)
				
				if self.options.degrees:
					precision = 1
				else:
					precision = 3
				
				add_line('joint_positions',precision)
				add_line('joint_velocities',precision)
				add_str_line('joint_command_types',3)
				add_line('joint_command_values',1)
				
				
				add_line('joint_set_point_positions',precision)
				add_line('joint_set_point_velocities',precision)

                                # ADDED
                                add_line('joint_avg_errs',3)
			
			header_fmt = ' '.join([header_field_name_format] + ([header_field_format.format(field_num=i) for i in xrange(len(self.fields['motor_types']))]))
			line_fmt = lambda field_format,precision: ' '.join([field_name_format] + ([field_format.format(field_num=i,precision=precision) for i in xrange(len(self.fields['motor_types']))]))
			
			if self.motors_enabled:
				self.add_line('\n------------------------------MOTORS------------------------------')
				self.add_line(header_fmt.format(*(self.fields['motor_types']),field_name=''))
				add_line('motor_positions',1)
				add_line('motor_velocities',1)
				add_line('motor_torques',3)
				
				add_line('motor_set_point_positions',1)
				add_line('motor_set_point_velocities',1)
				
                                # ADDED
                                add_line('motor_avg_errs',3)

				add_line('motor_enc_vals',0)
				add_line('motor_enc_offsets',0)
				add_line('motor_dac_cmds',0)
			''
			
			#  uint8 input_pins
			#  uint8 self.output_pins
		
		os.system('clear')
		print self.output
	
	def array_state_callback(self,msg):
            pass

        def get_joint_errors(self):
            #self.jointErrorLock.acquire()
	    l = None
	    if self.fields.has_key('joint_errs') and self.justStoppedAccum:
                l = list(self.fields['joint_errs'])
		self.justStoppedAccum = False
	    #self.jointErrorLock.release()
	    return l

if __name__ == '__main__':
    rospy.init_node('print_state',anonymous=True)
	
    parser = optparse.OptionParser()
	
    parser.add_option('-d','--degrees',action='store_true',default=True)
    parser.add_option('-r','--radians',action='store_false',dest='degrees')
    
    parser.add_option('--rate',type='float',default=10.)
	
    parser.add_option('-j','--joints',action='store_true',default=None)
    parser.add_option('-m','--motors',action='store_true',default=None)
	
    parser.add_option('--arm',action='append')
	
    (options,args) = parser.parse_args(rospy.myargv())
	
    if options.joints is None and options.motors is None:
        options.joints = True
	options.motors = True
	    
    if len(args) > 1:
        enabled = set(args[1:])
    else:
        enabled = set()
	
    printer = StatePrinter(options=options,enabled=enabled)
    
    sub = rospy.Subscriber('raven_state',RavenState,printer.state_callback)
    #sub = rospy.Subscriber('raven_state/array',printer.array_state_callback)
	
    numJoints = 9

    #fig = pylab.figure('Joint Errors')
    #graphs = fig.add_subplot(1,1,1) 

    fig, graphs = pylab.subplots(numJoints, sharex=True, sharey=False)

    pylab.show(block=False)

    #mng = pylab.get_current_fig_manager()
    #mng.frame.Maximize(True)

    while not rospy.is_shutdown():

        errs = printer.get_joint_errors()
	if errs != None:
            for index in range(numJoints):
                length = len(errs[index])
		
		if length == 0:
                    continue
		
                JE = np.array(errs[index])
		X = np.array(range(length))
		
		graphs[index].clear()
		
		graphs[index].plot(X,JE)
		graphs[index].annotate(str(errs[index][-1]), (length-1,errs[index][-1]), xycoords='data')
		
	    pylab.pause(.05)
	    fig.canvas.draw()
	    pylab.draw()
	"""
            je0 = np.array(errs[0])
	    X = np.array(range(len(errs[0])))
	    graph.clear()
            graph.plot(X, je0)
	    pylab.draw()
	"""
	rospy.sleep(.02)
         

    pylab.show()

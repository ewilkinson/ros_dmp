# -*- coding: utf-8 -*-
"""
Created on Sat Jun 14 15:55:10 2014

@author: ew
"""
import rosbag
import numpy as np
from pylab import *
from smooth import *

class JointBagLoader(object):
    def __init__(self, num_joints=12):
        self.num_joints = 12
        self.joint_positions = np.zeros((self.num_joints,1))
        
    def load(self, bag_loc, joint_mask=None, iss=0, ise=-1, smooth_dervs=True, start_at_zero=True):
        """ 
        Loads the joint information from a bag file. Assumes 12 joints
        
        Parameters 
        ------------
        bag_loc str: The location of the bag file on the file system
        
        joint_mask array-like: The indexes of the joints which we are interested in
        
        iss int: 
            (optional) The start index which to slice the joint data 
        
        ise int: 
            (optional) The end index which to slice to joint data

        smooth_dervs bool: 
            (optional) Whether to smooth the resulting velocity and acceleration information. Defaults to True
        
        start_at_zero bool:
            (optional) whether to initialize the joint and self.time data to begin at 0. Defaults to True
        """
        if not isinstance(bag_loc, str):
            raise ValueError('Bag location must be of type string and point to a file.')
        
        if joint_mask is None:
            self.joint_mask = np.arange(0,self.num_joints,1,dtype='int16')
        else:
            self.joint_mask = joint_mask
            
        bag = rosbag.Bag(bag_loc)
        topic_id = '/uBot/joint_positions'
            
        self.joint_positions = np.zeros((self.num_joints,1))
        self.time = np.zeros(1)

        for topic, msg, t in bag.read_messages(topics=[topic_id]):
        	self.joint_positions = np.column_stack((self.joint_positions, msg.positions))
        	self.time = np.column_stack((self.time, t.to_sec()))
        
        # Get rid of leading zeros and apply mask
        self.joint_positions = self.joint_positions[self.joint_mask,:]
        self.joint_positions = self.joint_positions[:,1:] # get rid of the leading zero
        self.time = self.time[0,1:]
        
        self.joint_positions = self.joint_positions[:, iss:ise]
        self.time = self.time[iss:ise]
        
        if start_at_zero:
            # initialize self.time and joint positions so that they start at 0
            self.time = self.time - self.time[0] 
            self.joint_positions = self.joint_positions - self.joint_positions[0]
        
        # dividing by the average self.time gives a less noisy signal. Typically the std is very, very low ~1E-5
        dt = np.diff(self.time)
        avg_dt = np.mean(dt)
        self.joint_vels = np.diff(self.joint_positions) / avg_dt
        
        # add a zero to the front of every row to make it the same size as positions
        self.joint_vels = np.hstack((np.zeros((self.joint_vels.shape[0], 1)), self.joint_vels))
        
        # perform smoothing on the signal 
        if smooth_dervs:
            for i in range(self.joint_vels.shape[0]):
            	self.joint_vels[i] = smooth(self.joint_vels[i])
             
        self.joint_accs = np.diff(self.joint_vels) / avg_dt
        
        # add a zero to the front of every row to make it the same size as positions
        self.joint_accs = np.hstack((np.zeros((self.joint_accs.shape[0], 1)), self.joint_accs))
        
        # perform smoothing on the signal 
        if smooth:
            for i in range(self.joint_accs.shape[0]):
            	self.joint_accs[i] = smooth(self.joint_accs[i])
             
        return self.time.copy(), self.joint_positions.copy(), self.joint_vels.copy(), self.joint_accs.copy()

    def clear(self):
        self.joint_positions = None
        self.joint_vels = None
        self.joint_accs = None
        self.time = None
        
    def plot_info(self, title_heading='Bag Joint Info', save_figure=False, file_loc=None):
        joint_lookup =['R_WHEEL', 'L_WHEEL','TORSO', 'R_TILT', 'L_TILT','R_PAN','L_PAN','R_TWIST','L_TWIST','R_ELBOW','L_ELBOW','HEAD_TILT']
        fig = plt.figure(1)
        ax  = fig.add_subplot(311)
        
        # this gives different colors for each line - http://matplotlib.sourceforge.net/examples/pylab_examples/show_colormaps.html
        colormap = plt.cm.Paired
        gca().set_color_cycle([colormap(i) for i in linspace(0, .9, self.joint_positions.shape[0])])
        
        hold(True)
        for i in range(self.joint_positions.shape[0]):
            ax.plot(self.time, self.joint_positions[i], label=joint_lookup[self.joint_mask[i]]+' position')
        title(title_heading)
        hold(False)
        ax.legend(loc = 'center left', bbox_to_anchor = (1.0, 0.5))
        ax2  = fig.add_subplot(312)
        # this gives different colors for each line - http://matplotlib.sourceforge.net/examples/pylab_examples/show_colormaps.html
        colormap = cm.Paired
        gca().set_color_cycle([colormap(i) for i in linspace(0, .9, self.joint_vels.shape[0])])
        hold(True)
        for i in range(self.joint_vels.shape[0]):
            ax2.plot(self.time, self.joint_vels[i], label=joint_lookup[self.joint_mask[i]]+' velocity')
        hold(False)
        ax2.legend(loc = 'center left', bbox_to_anchor = (1.0, 0.5))
        ax3  = fig.add_subplot(313)
        # this gives different colors for each line - http://matplotlib.sourceforge.net/examples/pylab_examples/show_colormaps.html
        colormap = cm.Paired
        gca().set_color_cycle([colormap(i) for i in linspace(0, .9, self.joint_accs.shape[0])])
        hold(True)
        for i in range(self.joint_accs.shape[0]):
            ax3.plot(self.time, self.joint_accs[i], label=joint_lookup[self.joint_mask[i]]+' acceleration')
        hold(False)
        ax3.legend(loc = 'center left', bbox_to_anchor = (1.0, 0.5))
        plt.show()

        if save_figure and file_loc is not None:
            fig.savefig(file_loc)
	
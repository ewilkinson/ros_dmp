import numpy as np
from pylab import *
from scipy import signal

import dmp.discrete_dmp as discrete_dmp
import dmp.rhythmic_dmp as rhythmic_dmp
import dmp.serializer_dmp as serializer_dmp
import dmp.lib_manager as lib_manager
from joint_bag_loader import *

bag_loc = './dmp/bags/test.bag'
show_figure = False
train_discrete_dmps = False
train_rhythmic_dmps = True

# define the slicing parameters which will extract the correct data from the bag file
joint_mask = [3,4,5,6,7,8,9,10]
index_slice_start = 2303
index_slice_end = 7100
#index_slice_end = -1

joint_loader = JointBagLoader()
time, joint_positions, joint_vels, joint_accs = joint_loader.load(bag_loc,joint_mask, iss=index_slice_start, ise=index_slice_end)

if show_figure:
    joint_loader.plot_info()
    
joint_loader.clear()

lib_mgr = lib_manager.DMPLibraryManager('./dmp/library.yaml', './dmp/pickles/')

if train_discrete_dmps or train_rhythmic_dmps:
    num_bfs = [500]
    
    # calculate the dt so that the total run time is 1
    dt = np.diff(time)
    avg_dt = np.mean(dt)
    for ii, bfs in enumerate(num_bfs):
        
        if train_discrete_dmps:
            dmp = discrete_dmp.DiscreteDMP(ID='do_the_twist', dmps=len(joint_mask), bfs=bfs, dt=avg_dt, run_time=time[-1])
        elif train_rhythmic_dmps:
            dmp = rhythmic_dmp.RhythmicDMP(ID='do_the_twist', dmps=len(joint_mask), bfs=bfs, dt=avg_dt, run_time=time[-1])
        
        dmp.imitate_path(y_des=joint_positions)   
        dmp.reset_state()
        y_track,dy_track,ddy_track = dmp.rollout(tau=1.0, timesteps=(time.shape[0]))
        
#        import matplotlib.pyplot as plt
#        plt.figure()
#        plt.subplot(211)
#        plt.plot(dmp.db_x, dmp.db_psi[1:,:])
#        plt.title('psi_track')
#        
#        plt.figure()
#        plt.subplot(211)
#        plt.plot(dmp.db_x, np.max(dmp.db_psi, axis=1)[1:])
#        plt.title('psi_sum')
        
        #lib_mgr.writeDMP(dmp, 'Grasping', 'Box')
        
#        serializer = serializer_dmp.SerializerDMP(root_folder='./dmp/pickles/')
#        serializer.write(dmp, 'Test/Test/')
        #dmp_load = serializer.load('Test/Test/do_the_twist.p')
        #y_track,dy_track,ddy_track = dmp_load.rollout(tau=1.0, timesteps=time.shape[0])

        fig = plt.figure(2)
        for i in range(y_track.shape[0]):
            plt.subplot(211)
            plt.plot(time, y_track[i,:], lw=2)
            plt.subplot(212)
            plt.plot(time, dy_track[i,:], lw=2)
        
    for i in range(joint_positions.shape[0]):
        plt.subplot(211)
        a = plt.plot(time, joint_positions[i], 'r--', lw=2)
        plt.title('DMP imitate path')
        plt.xlabel('time (s)')
        plt.ylabel('system trajectory')
        plt.legend([a[0]], ['desired path'], loc='lower right')
        plt.subplot(212)
        b = plt.plot(time, joint_vels[i], 'r--', lw=2)
        plt.title('DMP imitate path')
        plt.xlabel('time (s)')
        plt.ylabel('system trajectory')
        plt.legend(['%i BFs'%i for i in num_bfs], loc='lower right')
    
    plt.tight_layout()
    plt.show()
    
    



	

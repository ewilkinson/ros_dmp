'''
Copyright (C) 2013 Travis DeWolf

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

from dmp import DMPs

import numpy as np

class DiscreteDMP(DMPs):
    """An implementation of discrete DMPs"""

    def __init__(self, **kwargs): 
        """
        """

        # call super class constructor
        super(DiscreteDMP, self).__init__(pattern='discrete', **kwargs)

        self.gen_centers()

        # set variance of Gaussian basis functions
        # trial and error to find this spacing
        self.h = np.ones(self.bfs) * self.bfs**1.5 / self.c

        self.check_offset()
        
    def gen_centers(self):
        """Set the centre of the Gaussian basis 
        functions be spaced evenly throughout run time"""

        # desired spacings along x
        # need to be spaced evenly between 1 and exp(-ax)
        # lowest number should be only as far as x gets 
        #self.c = np.logspace(0,-self.cs.ax*self.cs.run_time,num=self.bfs,base=np.e)
        self.c = np.logspace(0,-self.cs.ax*1,num=self.bfs,base=np.e)
        
        
#        first = np.exp(-self.cs.ax*self.cs.run_time)
#        last = 1.05 - first
#        des_c = np.linspace(first,last,self.bfs) 
#        
#        self.c = -np.log(des_c)

    def gen_front_term(self, x, dmp_num):
        """Generates the diminishing front term on 
        the forcing term.

        x float: the current value of the canonical system
        dmp_num int: the index of the current dmp
        """
        
        return x * (self.goal[dmp_num] - self.y0[dmp_num])

    def gen_goal(self, y_des): 
        """Generate the goal for path imitation. 
        For rhythmic DMPs the goal is the average of the 
        desired trajectory.
    
        y_des np.array: the desired trajectory to follow
        """

        return y_des[:,-1].copy()
        
    def gen_psi(self, x):
        """Generates the activity of the basis functions for a given 
        canonical system rollout. 
        
        x float, array: the canonical system state or path
        """
        if isinstance(x, np.ndarray):
            x = x[:,None]
            
        return np.exp(-self.h * (x - self.c)**2)

    def gen_weights(self, f_target):
        """Generate a set of weights over the basis functions such 
        that the target forcing term trajectory is matched.
        
        f_target np.array: the desired forcing term trajectory
        """

        # calculate x and psi   
        x_track = self.cs.rollout()
        psi_track = self.gen_psi(x_track)
        #self.psi_track_db = psi_track

        #efficiently calculate weights for BFs using weighted linear regression
        self.w = np.zeros((self.dmps, self.bfs))
        for d in range(self.dmps):
            # spatial scaling term
            k = (self.goal[d] - self.y0[d])
            for b in range(self.bfs):
                numer = np.sum(x_track * psi_track[:,b] * f_target[:,d])
                denom = np.sum(x_track**2 * psi_track[:,b])
                self.w[d,b] = numer / (k * denom)
                
        '''# plot the basis function activations
        import matplotlib.pyplot as plt
        plt.figure()
        plt.subplot(211)
        plt.plot(psi_track)
        plt.title('psi_track')

        # plot the desired forcing function vs approx
        plt.subplot(212)
        plt.plot(f_target[:,0])
        plt.plot(np.sum(psi_track * self.w[0], axis=1))
        plt.legend(['f_target', 'w*psi'])
        plt.tight_layout()
        plt.show()'''

#==============================
# Test code
#==============================
if __name__ == "__main__":

    # test normal run
    dmp = DiscreteDMP(ID='no_forcing',dmps=1, bfs=10, w=np.zeros((1,10)))
    y_track,dy_track,ddy_track = dmp.rollout()
    
    import matplotlib.pyplot as plt
    plt.figure(1, figsize=(6,3))
    plt.plot(np.ones(len(y_track[0,:]))*dmp.goal, 'r--', lw=2)
    plt.plot(y_track[0,:], lw=2)
    plt.title('DMP system - no forcing term')
    plt.xlabel('time (ms)')
    plt.ylabel('system trajectory')
    plt.legend(['goal', 'system state'], loc='lower right')
    plt.tight_layout()
    plt.show()

    # test imitation of path run
    plt.figure(2, figsize=(6,4))
    #num_bfs = [10, 30, 50, 100, 10000]
    num_bfs = [10]

    # a straight line to target
    path1 = np.sin(np.arange(0,1,.01)*10)
    # a strange path to target
    path2 = np.zeros(path1.shape)
    path2[(len(path2) / 2.):] = .5 

    for ii, bfs in enumerate(num_bfs):
        dmp = DiscreteDMP(ID='test_forcing',dmps=2, bfs=bfs)

        dmp.imitate_path(y_des=np.array([path1, path2]))
        # change the scale of the movement
        dmp.goal[0] = 3; dmp.goal[1] = 2

        y_track,dy_track,ddy_track = dmp.rollout(tau=1.0)

        plt.figure(2)
        plt.subplot(211)
        plt.plot(y_track[0,:], lw=2)
        plt.subplot(212)
        plt.plot(y_track[1,:], lw=2)

    plt.subplot(211)
    a = plt.plot(path1 / path1[-1] * dmp.goal[0], 'r--', lw=2)
    plt.title('DMP imitate path')
    plt.xlabel('time (ms)')
    plt.ylabel('system trajectory')
    plt.legend([a[0]], ['desired path'], loc='lower right')
    plt.subplot(212)
    b = plt.plot(path2 / path2[-1] * dmp.goal[1], 'r--', lw=2)
    plt.title('DMP imitate path')
    plt.xlabel('time (ms)')
    plt.ylabel('system trajectory')
    plt.legend(['%i BFs'%i for i in num_bfs], loc='lower right')

    plt.tight_layout()
    plt.show()
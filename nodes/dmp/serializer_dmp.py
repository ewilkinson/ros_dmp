# -*- coding: utf-8 -*-
"""
Created on Tue Jun 10 20:37:42 2014

@author: ewilkinson
"""
import cPickle as pickle
from dmp import *
from discrete_dmp import *
from rhythmic_dmp import *

class SerializerDMP:
    def __init__(self, root_folder=None):
        self.root_folder = root_folder
        
    def write(self, dmp, file_loc):
        """ Writes the object DMP to a pickle binary file
        Uses the DMP ID as a file name
    
        Parameters
        -------------
        dmp DMPs: The dmp you want to serialize
        file_loc str: The file location relative to the root folder
        
        Notes do not add
        """
        
        dmp.reset_state()
        
        if self.root_folder is not None:
            file_loc = self.root_folder + file_loc + dmp.ID +'.p'
        
        pickle.dump(dmp, open(file_loc, 'wb'))
        
    def load(self, file_loc):
        """ Returns a DMP for a provided dmp_id
        Be sure to catch IOError exceptions if the file 
        cannot be found.
        
        """
        if file_loc is None:
            return None
            
        if self.root_folder is not None:
            file_loc = self.root_folder + file_loc
        
        print 'Serializer Loading : ', file_loc
        return pickle.load(open(file_loc, 'rb'))
        
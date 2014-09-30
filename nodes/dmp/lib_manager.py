# -*- coding: utf-8 -*-
"""
Created on Sat Jun 14 20:50:56 2014

@author: ewilkinson
"""
import yaml
from serializer_dmp import *
import sys, traceback

class DMPLibraryManager(object):
    """
    Library manager for DMPs. Library consists of sections and shelves. 
    Manager can only manage one library yaml file at a time
    
    Parameters
    ------------
    library_file_loc str: 
        location of the library yaml file.
        
    """
    def __init__(self, library_file_loc, root_pickle_folder):
        self._dmps = {}
        self.serializer = SerializerDMP(root_folder=root_pickle_folder)
        self.loadLibraryYAML(library_file_loc)
        
    def loadLibraryYAML(self, library_file_loc):
        self._library_file_loc = library_file_loc
        self._lib = yaml.load(open(library_file_loc, 'r')) 
    
    def listSections(self):
        """
        List all the sections in the library
        """
        return self._lib.keys()
        
    def listShelves(self, section_name):
        if not self._lib.has_key(section_name):
            raise Exception('Library did not contain section : ' + str(section_name))
        
        return self._lib[section_name].keys()
        
    def loadEntireLibrary(self):
        for section in self._lib.keys():
            self.loadSection(section)
    
    def loadShelf(self, section_name, shelf_name):
        for dmp_id in self._lib[section_name][shelf_name]:
            file_loc = section_name +'/' + shelf_name + '/' + dmp_id
            dmp_lib_id = self._to_lib_id(section_name, shelf_name, dmp_id)
            try:
                dmp = self.serializer.load(file_loc)
            except ImportError as e:
                print 'Serializer error : ', e.message
                ex_type, ex, tb = sys.exc_info()
                traceback.print_tb(tb)
                continue
            
            print 'Completed Load DMP : ', dmp_lib_id  
            self._dmps[dmp_lib_id] = dmp
                
    def loadSection(self, section_name):
        shelves = self.listShelves(section_name)
        for shelf in shelves:
            self.loadShelf(section_name, shelf)
            
    def getDMP(self, dmp_lib_id):
        """
        Parameters
        -------------
        dmp_lib_id str: 
            The library id for the dmp. Of the form Section::Shelf::DMP_ID
        """
        return self._dmps[dmp_lib_id] if self._dmps.has_key(dmp_lib_id) else None
        
    def writeDMP(self, dmp, section_name, shelf_name):
        """
        Serializes the dmp and adds it to the currently loaded library. 
        """
        self.serializer.write(dmp, section_name+'/'+shelf_name+'/')
        self._dmps[self._to_lib_id(section_name, shelf_name, dmp.ID)] = dmp
        
        if not self._lib.has_key(section_name):
            self._lib[section_name] = {}
        
        if not self._lib[section_name].has_key(shelf_name):
            self._lib[section_name][shelf_name] = [dmp.ID + '.p']
        else:
            self._lib[section_name][shelf_name].append(dmp.ID + '.p')
            
        with open(self._library_file_loc, 'w') as yaml_file:
            yaml_file.write( yaml.dump(self._lib, default_flow_style=False))
        
    def removeDMP(self, dmp_lib_id):
        """
        Removes a DMP from the library in memory. Does not remove from filesystem
        
        Parameters
        -------------
        dmp_lib_id str: 
            The library id for the dmp. Of the form Section::Shelf::DMP_ID
        """
        self._dmps[dmp_lib_id] = None
        
    def _to_lib_id(self, section_name, shelf_name, dmp_id):
        if dmp_id.find('.p') is -1:
            dmp_id = dmp_id + '.p'
            
        return section_name + '/' + shelf_name + '/' + dmp_id
        
        
    
if __name__ == '__main__':
    lib_mgr = DMPLibraryManager('library.yaml', './pickles/')
    sections = lib_mgr.listSections()
    print sections
    print lib_mgr.listShelves(sections[0])
    
    lib_mgr.loadSection('Test')
    
    dmp = lib_mgr.getDMP('Test/Test/test.p')
    dmp.ID = 'Library_Test'
    lib_mgr.writeDMP(dmp, 'Test', 'Test')
    

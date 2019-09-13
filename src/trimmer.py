#!/usr/bin/python

import os
from math import pi
import ctypes as ct


print('Currently working on {}'.format(os.getcwd()))
uav_name = 'skywalker_2013'

class Trimmer():
    trim_input = []
    trim_input_2 = []
    dll = None
    trimFunc = None
    trimFunc2 = None
    input_type = ct.c_double * 6
    output_type = ct.c_double * 6

    def __init__(self, uav_name):
        self.dll = ct.cdll.LoadLibrary('/home/george/ros_workspaces/uav_ftc/src/last_letter/last_letter_lib/build/liblib_trimmer.so')
        self.dll.Trimmer_new.argtypes = [ct.c_char_p]
        self.dll.Trimmer_new.restype = ct.c_void_p
        self.dll.find_trim.argtypes = [ct.c_void_p, ct.c_double * 6]
        self.dll.find_trim.restype = ct.POINTER(ct.c_double)
        self.dll.find_trim_2.argtypes = [ct.c_char_p, ct.c_double * 6]
        self.dll.find_trim_2.restype = ct.POINTER(ct.c_double)

        self.obj = self.dll.Trimmer_new(uav_name)

        print('Got Trimmer object of type {}'.format(type(self.obj)))
        self.trimFunc = self.dll.find_trim
        # self.trimFunc.argtypes = [type(self.obj), ct.c_double * 6]
        # self.trimFunc.argtypes = None * 2
        # self.trimFunc.argtypes[1] = ct.c_double * 6
        # self.trimFunc.argtypes = [ct.c_int, ct.c_double * 6]
        self.trimFunc2 = self.dll.find_trim_2

    def find_trim_input(self, trim_states):
        print('Querying trim point')
        # self.trim_input_2 = self.trimFunc(self.obj, trim_states, self.trim_input)
        self.trim_input = self.trimFunc(self.obj, trim_states)

    def find_trim_input_2(self, trim_states):
        print('Querying trim point 2')
        # self.trim_input_2 = self.trimFunc(self.obj, trim_states, self.trim_input)
        self.trim_input_2 = self.trimFunc2(uav_name, trim_states)

trimmer = Trimmer(uav_name)
print('Successfully loaded Trimmer object')

# trim_states = [0, 0, 1*pi/180, 10, 1*pi/180, 0]
trim_states = Trimmer.input_type(0, 1*pi/180, 10, 1*pi/180, 0, 0)

# trimmer.find_trim_input_2(trim_states)
trimmer.find_trim_input(trim_states)
print("Trim found: {}".format(trimmer.trim_input))
ctrl = trimmer.trim_input
print("Trim found 2: {}, {}, {}, {}, {}, {}".format(ctrl[0], ctrl[1], ctrl[2], ctrl[3], ctrl[4], ctrl[5]))

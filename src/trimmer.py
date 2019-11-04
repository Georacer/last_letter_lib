#!/usr/bin/python

import os
import time
from math import pi
import numpy as np
import ctypes as ct

print('Currently working on {}'.format(os.getcwd()))

class Trimmer():
    dll = None
    obj = None
    trim_func = None
    trim_func_2 = None
    input_type = ct.c_double * 6
    output_type = ct.POINTER(ct.c_double)
    trim_input = None

    def __init__(self, uav_name):
        # Load the lib_trimmer dll
        self.dll = ct.cdll.LoadLibrary('/home/george/ros_workspaces/uav_ftc/src/last_letter/last_letter_lib/build/liblib_trimmer.so')

        # Set the exposed functions data types
        self.dll.trimmer_new.argtypes = [ct.c_char_p]
        self.dll.trimmer_new.restype = ct.c_void_p
        self.trim_func = self.dll.find_trim
        self.dll.find_trim.argtypes = [ct.c_void_p, self.input_type]
        self.dll.find_trim.restype = self.output_type
        self.trim_func_2 = self.dll.find_trim_2
        self.dll.find_trim_2.argtypes = [ct.c_char_p, self.input_type]
        self.dll.find_trim_2.restype = self.output_type
        self.print_optimal_result = self.dll.print_optimal_result
        self.dll.print_optimal_result.argtypes = [ct.c_void_p]
        self.dll.print_optimal_result.restype = ct.c_double

        self.uav_name = uav_name
        self.obj = self.dll.trimmer_new(uav_name)

    # Convert trim states from a numpy array of [phi, theta, Va, alpha, beta ,r]
    # to a suitable input type for find_trim()
    def convert_trim_states(self, trim_states_np):
        return self.input_type(*trim_states_np)

    def convert_trim_controls(self, trim_ctrls_ct):
        trim_ctrls = np.zeros(6)
        for i in range(6):
            trim_ctrls[i] = trim_ctrls_ct[i]
        return trim_ctrls

    def find_trim_input(self, trim_states):
        trim_states_ct = self.convert_trim_states(trim_states)
        trim_input_ct = self.trim_func(self.obj, trim_states_ct)
        self.trim_input = self.convert_trim_controls(trim_input_ct)
        return self.trim_input

    # Do not use this function, it loads a new Trimmer object every time
    def find_trim_input_2(self, trim_states):
        trim_states_ct = self.convert_trim_states(trim_states)
        trim_input_ct = self.trim_func_2(self.uav_name, trim_states_ct)
        self.trim_input = self.convert_trim_controls(trim_input_ct)
        return self.trim_input

    def print_result(self):
        self.print_optimal_result(self.obj)

if __name__ == '__main__':
    print('Testing trimmer module functionality')
    uav_name = 'skywalker_2013'
    trimmer = Trimmer(uav_name)
    print('Successfully loaded Trimmer object')

    trim_states = np.array([0, 0*pi/180, 15, -0.05, 0, 0])
    # phi, theta, Va, alpha, beta, r
    print('Obtaining trim input: 1000 repetitions')
    t_start = time.time()
    for i in range(1000):
        trim_ctrls = trimmer.find_trim_input(trim_states)
    t_end = time.time()

    print("Trim found:\n" + "{}".format(trim_ctrls))
    trimmer.print_result()
    print("Time required: {}s".format(t_end-t_start))

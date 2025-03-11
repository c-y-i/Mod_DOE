################################################################################
# Author: Greg Campbell
#
# Including code from 2017 ROBOTIS CO., LTD. - Author: Ryu Woon Jung (Leon)
#
# *********     Experimental Test Procedure      *********
#
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Current data from an Adafruit INA260 breakout board via ESP32 Pico D4

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from Serial_Comms import *

import time                                    # Use some timers
import serial                                  # Import serial library
import numpy as np                             # Import numpy
import csv                                     # Import csv 
import datetime                                # Import datetime
import pickle                                  # Import pickle

# ESP32 Pico D4 features
esp_serial = '/dev/ttyUSB1'
esp_baud = 115200

# Dictionary details
dict_file = 'force_dict.pkl'
in_mass = 1 #kg
in_material = 'PLA'

#################################################
# Main Loop
#################################################

# Set control type
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# flush serial buffer
flush(esp_serial, baud_rate=esp_baud)

# create an empty array
esp_inputs = np.zeros([0,2]).astype(int)

while(1):
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    start_time = time.time()
    while(time.time()-start_time < 2):
        # read state
        in_state = read_state(esp_serial, baud_rate=esp_baud, expected_len=2)
        try:
            in_load = float(in_state[1][:-3]) # N
            # in_time = float(in_state[0])
        except:
            in_current = 0
        # append
        esp_inputs = np.vstack((esp_inputs,[int((time.time()-start_time)*1000),in_load]))

# add to existing dictionary
key_name = str(in_mass) + 'kg_' + str(in_material)

os.chdir('Mod_DOE')
# dict_set = {}
dict_set = pickle.load(open(dict_file, "rb"))
# see if key exists
if key_name in dict_set.keys():
    print("Key exists, appending")
    dict_set[key_name].append(all_inputs)
else:
    dict_set[key_name] = [all_inputs] # save as a list
pickle.dump(dict_set, open(dict_file, "wb"))

# shape : [[nx1,nx1],nx1,nx1] - n is number of samples
# shape : [mxn,mxn,mxn,mxn ... ] - m is number of TRIALS (... is number of reported values)
    # concatenate over m trials - > [nx1,nx1,nx1,nx1] (samples)
        # report single value: [1,0] - cost (i.e. max(current))
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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from Serial_Comms import *

import time                                    # Use some timers
import serial                                  # Import serial library
import numpy as np                             # Import numpy
import csv                                     # Import csv 
import datetime                                # Import datetime
import pickle                                  # Import pickle

# ESP32 Pico D4 features
esp_serial = '/dev/ttyUSB0' # CHECK - may be 0 or 1
esp_baud = 115200

# Dictionary details
dict_file = 'test_dict.pkl'
in_num = 393

#############################################################################
# Dynamixel Features
#############################################################################

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_PRESENT_SPEED      = 38
ADDR_MX_PRESENT_LOAD       = 40
ADDR_MX_LED                = 25
CW_ANGLE_LIMIT             = 6
CCW_ANGLE_LIMIT            = 8


# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 35                # Dynamixel ID : 1
BAUDRATE                    = 3000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB1'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
SPEED = [100, 1124]
STOP = 0


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

#############################################################################
# Dynamixel Control Functions
#############################################################################
# Toggle LED
def dxl_set_LED(on_off_bool,portHandler = portHandler, packetHandler = packetHandler, DXL_ID = DXL_ID, ADDR_MX_LED = ADDR_MX_LED):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_LED, on_off_bool)
    if dxl_comm_result != COMM_SUCCESS:
        print("LED ERROR %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("LED ERROR %s" % packetHandler.getRxPacketError(dxl_error))
    return

# Velocity Control - wheel mode only
def dxl_move_speed(speed,portHandler = portHandler, packetHandler = packetHandler, DXL_ID = DXL_ID, ADDR_MX_MOVING_SPEED = ADDR_MX_MOVING_SPEED):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, speed)
    if dxl_comm_result != COMM_SUCCESS:
        print("Speed ERROR %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("Speed ERROR %s" % packetHandler.getRxPacketError(dxl_error))
    return

# Position Control - joint or multiturn mode only
def dxl_move_position(position,portHandler = portHandler, packetHandler = packetHandler, DXL_ID = DXL_ID, ADDR_MX_GOAL_POSITION = ADDR_MX_GOAL_POSITION):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, position)
    if dxl_comm_result != COMM_SUCCESS:
        print("Position ERROR %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("Position ERROR %s" % packetHandler.getRxPacketError(dxl_error))
    return

# Set control type - joint/wheel/multiturn
def dxl_set_Control(control,portHandler = portHandler, packetHandler = packetHandler, DXL_ID = DXL_ID, CW_ANGLE_LIMIT = CW_ANGLE_LIMIT, CCW_ANGLE_LIMIT = CCW_ANGLE_LIMIT):
    # Set CW & CCW Limit to 0 (for continuous rotation)
    if control == 'joint':
        CW_VAL = 1
        CCW_VAL = 4095
    elif control == 'wheel':
        CW_VAL = 0
        CCW_VAL = 0
    elif control == 'multiturn':
        CW_VAL = 4095
        CCW_VAL = 4095
    else:
        print("Invalid control type")
        return

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, CW_ANGLE_LIMIT, CW_VAL)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, CCW_ANGLE_LIMIT, CCW_VAL)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    return

# Enable Dynamixel Torque
def dxl_set_Torque_enable(on_off_bool, portHandler = portHandler, packetHandler = packetHandler, DXL_ID = DXL_ID, ADDR_MX_TORQUE_ENABLE = ADDR_MX_TORQUE_ENABLE):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, on_off_bool)
    if dxl_comm_result != COMM_SUCCESS:
        print("Torque ERROR %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("Torque ERROR %s" % packetHandler.getRxPacketError(dxl_error))
    return

# Read Dynamixel at given adress (acceptable addresses are 2-byte only)
# Currently works for ADDR_MX_PRESENT_POSITION, ADDR_MX_PRESENT_SPEED, ADDR_MX_PRESENT_LOAD
def dxl_read_address(ADDR, portHandler = portHandler, packetHandler = packetHandler, DXL_ID = DXL_ID):
    dxl_address_value, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR)
    if dxl_comm_result != COMM_SUCCESS:
        print("Position ERROR %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("Position ERROR %s" % packetHandler.getRxPacketError(dxl_error))
    return dxl_address_value


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


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

dxl_set_Control('wheel')
dxl_set_Torque_enable(True)

# flush serial buffer
flush(esp_serial, baud_rate=esp_baud)

# create an empty array
dxl_inputs = np.zeros([0,2]).astype(int)
esp_inputs = np.zeros([0,1]).astype(int)

while(1):
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    # # Write goal speed
    dxl_set_LED(True)
    dxl_move_speed(SPEED[index])
    start_time = time.time()
    while(time.time()-start_time < 2):
        # Too slow to read speed & position as well (~15ms per read (!!))
        # speed = dxl_read_address(ADDR_MX_PRESENT_SPEED)
        # pos = dxl_read_address(ADDR_MX_PRESENT_POSITION)

        # Read load
        load = int(dxl_read_address(ADDR_MX_PRESENT_LOAD))

        # Read current
        in_state = read_state(esp_serial, baud_rate=esp_baud, expected_len=1) #4/17 update to len 1
        try:
            # in_current = float(in_state[1][:-3]) #get rid of 'mA' label
            in_current = float(in_state[0][2:]) #HACK - get rid of b'
            # print(in_current)
        except:
            # print("Error reading current")        print(in_state[0][2:]
            in_current = -1
        # append
        dxl_inputs = np.vstack((dxl_inputs,[int((time.time()-start_time)*1000),load]))
        esp_inputs = np.vstack((esp_inputs,[in_current]))
        # print("Speed: %d, Position: %d, Load: %d" % (speed, pos, load))

    dxl_set_LED(False)
    dxl_move_speed(STOP)

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

dxl_set_Torque_enable(False)

portHandler.closePort()

all_inputs = np.hstack((dxl_inputs,esp_inputs))

# add to existing dictionary
key_name = 'test' + str(in_num)

os.chdir('Mod_DOE')
# dict_set = {}
dict_set = pickle.load(open(dict_file, "rb"))
# see if key exists
if key_name in dict_set.keys():
    print("Key exists, appending")
    dict_set[key_name].append(all_inputs)
else:
    dict_set[key_name] = [all_inputs] # save as a list
    print("Key does not exist, creating new key")
pickle.dump(dict_set, open(dict_file, "wb"))


# shape : [[nx1,nx1],nx1,nx1] - n is number of samples
# shape : [mxn,mxn,mxn,mxn ... ] - m is number of TRIALS (... is number of reported values)
    # concatenate over m trials - > [nx1,nx1,nx1,nx1] (samples)
        # report single value: [1,0] - cost (i.e. max(current))
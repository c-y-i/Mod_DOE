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
import matplotlib.pyplot as plt
import matplotlib.cm as cm

# if os.name == 'nt':
#     import msvcrt
#     def getch():
#         return msvcrt.getch().decode()
# else:
#     import sys, tty, termios
#     fd = sys.stdin.fileno()
#     # old_settings = termios.tcgetattr(fd)
#     def getch():
#         try:
#             tty.setraw(sys.stdin.fileno())
#             ch = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from Serial_Comms import *

import time                                    # Use some timers
import serial                                  # Import serial library
import numpy as np                             # Import numpy
import csv                                     # Import csv 
import datetime                                # Import datetime
import pickle                                  # Import pickle
import threading                              # Import threading
from queue import Queue                   # Import queue

# ESP32 Pico D4 features
esp_serial = '/dev/ttyUSB0' # CHECK - may be 0 or 1
# esp_baud = 115200
esp_baud = 500_000 #ESP default baudrate : 115200

# Dictionary details TODO - check and delete.
dict_file = 'v2_test_dict.pkl'
in_num = 1

# file variables for recording
gear_rec_bool = True
valve_rec_bool = True

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
BAUDRATE                    = 3_000_000           # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB1'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

direction = 0
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

# read in the 'percent torque' value from the dynamixel, return to queue 'q'
def dxl_read_loads(q):
    load_vals = []
    timestamp = []
    init_time = time.time()
    while(gear_rec_bool):
        load = int(dxl_read_address(ADDR_MX_PRESENT_LOAD))
        load_vals.append(load)
        timestamp.append(int((time.time()-init_time)*1000))
    print(f'Posting load vals, shape: {len(load_vals)}')
    q.put( [timestamp, load_vals] )

# Read current from ESP32 serial, return to queue 'q'
def read_current(q, serial_port, baud_rate=esp_baud):
    raw = []
    timestamp = []
    init_time = time.time()
    loop_count = 0
    with serial.Serial(port=serial_port, baudrate=baud_rate, bytesize=8) as ser:
        while(gear_rec_bool):
            if ser.in_waiting > 0:
                line = ser.readline()
                # if line:
                raw.append(line)
                timestamp.append(int((time.time()-init_time)*1000))
            loop_count += 1

    current_vals = []
    for row in raw:
        try:
            line = row.decode('utf-8').rstrip()
        except:
            current_vals.append(-1)
            continue
        try:
            value = parse_string(line, float_only=True)[1] # 2 columns should be time, pressure
            current_vals.append(value)
        except:
            value = parse_string(line, float_only=True)
            if len(value):
                value = value[0]
                try:
                    current_vals.append(value)
                except:
                    current_vals.append(-1)
            else:
                current_vals.append(-1)
        

    print(f'Posting current vals, shape: {len(current_vals)}')
    q.put( [timestamp, current_vals, np.ones(len(timestamp))] ) # third column to differentiate currents


##############################################
# Plotting Function
##############################################
def plot_dict_key(in_dict, dict_key, save_loc = None, file_name = None):
    fig = plt.figure()
    # 2 subplots
    ax = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2)

    n = len(in_dict[dict_key])

    # set up colors
    base_c1 = cm.Reds
    base_c2 = cm.Blues
    color1 = [base_c1(i / n) for i in range(n)]
    color2 = [base_c2(i / n) for i in range(n)]

    # plot all the trials
    dxt_all_vals = []
    current_all_vals = []
    for i in range(n):
        current_time = np.array(in_dict[dict_key][i][1][0])
        current_vals = np.array(in_dict[dict_key][i][1][1])
        dxt_time = np.array(in_dict[dict_key][i][0][0])
        dxt_vals = np.array(in_dict[dict_key][i][0][1])

        if np.mean(dxt_vals) > 1024: # CW
            dxt_percent = (dxt_vals - 1024) / 1023
        else:
            dxt_percent =  - dxt_vals / 1023 #CCW
        ax.scatter(dxt_time, dxt_percent, c=color1[i], label='torque')
        ax2.scatter(current_time, current_vals, c=color2[i], label='current')
        dxt_all_vals.extend(dxt_percent)  # Collect all values
        current_all_vals.extend(current_vals)

    ax.set_xlabel('time (ms)')
    ax.set_ylabel('dxt torque (%)')
    ax2.set_xlabel('time (ms)')
    ax2.set_ylabel('current (mA)')
    current_ll = np.percentile(current_all_vals, 1)
    current_ul = np.percentile(current_all_vals, 99)
    dxt_ll = np.percentile(dxt_all_vals, 1)
    dxt_ul = np.percentile(dxt_all_vals, 99)
    ax.set_ylim([dxt_ll, dxt_ul])
    ax2.set_ylim([current_ll, current_ul])

    plt.tight_layout()
    if save_loc and file_name:
        save_path = f"{save_loc}/{file_name}"
        plt.savefig(save_path)  # Save the figure to the specified location

    plt.show()

def valve_plot_dict_key(in_dict, dict_key, save_loc = None, file_name = None):
    fig = plt.figure()
    # 1 subplot (for now)
    ax = fig.add_subplot(1, 1, 1)

    n = len(in_dict[dict_key])

    # set up colors
    base_c1 = cm.Reds
    base_c2 = cm.Blues
    color1 = [base_c1(i / n) for i in range(n)]
    color2 = [base_c2(i / n) for i in range(n)]

    # plot all the trials
    p1_all_vals = []
    for i in range(n):
        p1_time = np.array(in_dict[dict_key][i][0])
        p1_vals = np.array(in_dict[dict_key][i][1])

        ax.scatter(p1_time, p1_vals, c=color1[i], label='pressure')
        p1_all_vals.extend(p1_vals)

    ax.set_xlabel('time (ms)')
    ax.set_ylabel('p1 pressure (kPa)')
    # ax2.set_xlabel('time (ms)')
    # ax2.set_ylabel('p2 pressure (kPa)')
    p1_ll = np.percentile(p1_all_vals, 1)
    p1_ul = np.percentile(p1_all_vals, 99)

    ax.set_ylim([p1_ll, p1_ul])

    plt.tight_layout()
    if save_loc and file_name:
        save_path = f"{save_loc}/{file_name}"
        plt.savefig(save_path)  # Save the figure to the specified location

    plt.show()

#################################################
# Run **Gear** Experiment - Main Function
#################################################

def Run_Experiment(in_dict, RUNTIME = 2, in_key = 'test', DIR = direction, file_dir = None, file_name = None, save = True):
    # DIR bool sets direction
    if DIR:
        dir_str = 'CCW'
    else:
        dir_str = 'CW'
    
    # Set control type
    # Open port
    try:
        dxl_success =  portHandler.openPort()
    except:
        print("Failed to open the dxl port - try reconnecting USB's.")
        return
    if not dxl_success:
        print("Failed to open the dxl port - try reconnecting USB's.")
        return


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        # print("Succeeded to change the baudrate")
        pass
    else:
        print("Failed to change the baudrate")
        # print("Press any key to terminate...")
        # getch()
        return

    # Use continuous motion mode
    dxl_set_Control('wheel')
    dxl_set_Torque_enable(True)

    # create an empty array - TODO - delete?
    dxl_inputs = np.zeros([0,2]).astype(int)
    esp_inputs = np.zeros([0,1]).astype(int)

    ## Section for getch() control (doesn't work for function call)
    # while(1):
    #     print("Press any key to continue! (or press ESC to quit!)")
    #     if getch() == chr(0x1b):
    #         break
        
    print(f'Running motor {dir_str} for {RUNTIME} seconds ...')

    # use a Queue for thread-safe communication
    output_queue = Queue()

    thread_current = threading.Thread(target=read_current, args=(output_queue, esp_serial, esp_baud))
    thread_dxl = threading.Thread(target=dxl_read_loads, args=(output_queue,))

    global gear_rec_bool 
    gear_rec_bool = True
    # Start threads
    thread_current.start()

    # # Write goal speed
    dxl_set_LED(True)
    dxl_move_speed(SPEED[DIR])
    start_time = time.time()

    thread_dxl.start()

    # Wait for run
    try:
        while(time.time()-start_time < RUNTIME):
            pass

    except KeyboardInterrupt:
        print("Kernel interrupted! Stopping the function.")
        # Perform any cleanup if necessary
        dxl_set_LED(False)
        dxl_move_speed(STOP)
        dxl_set_Torque_enable(False)
        portHandler.closePort()

        gear_rec_bool = False
        if thread_current.is_alive():
            thread_current.join()
        if thread_dxl.is_alive():
            thread_dxl.join()
        return

    gear_rec_bool = False
    thread_dxl.join()
    read_queue1 = output_queue.get()

    dxl_set_LED(False)
    dxl_move_speed(STOP)

    # turn off gear_rec_bool and collect results
    thread_current.join()
    read_queue2 = output_queue.get()

    # make sure esp and dxl values are sorted correctly
    if len(read_queue2) == 2:
        dxl_inputs = read_queue2
        esp_inputs = read_queue1[:-1]
    else:
        dxl_inputs = read_queue1
        esp_inputs = read_queue2[:-1]

    # Change goal position
    if DIR == 0:
        direction = 1
    else:
        direction = 0

    dxl_set_Torque_enable(False)

    portHandler.closePort()

    all_inputs = [dxl_inputs,esp_inputs]

    # add to existing dictionary
    key_name = in_key

    dict_set = in_dict

    # see if key exists
    if key_name in dict_set.keys():
        print("Key exists, appending")
        dict_set[key_name].append(all_inputs)
    else:
        dict_set[key_name] = [all_inputs] # save as a list
        print("Key does not exist, creating new key")

    if save and file_dir and file_name:
        init_dir = os.getcwd()
        os.chdir(file_dir)
        pickle.dump(dict_set, open(file_name, "wb"))
        os.chdir(init_dir)

    return dict_set

    # shape : [[nx1,nx1],nx1,nx1] - n is number of samples
    # shape : [mxn,mxn,mxn,mxn ... ] - m is number of TRIALS (... is number of reported values)
        # concatenate over m trials - > [nx1,nx1,nx1,nx1] (samples)
            # report single value: [1,0] - cost (i.e. max(current))

################################################
# Pressure sensor helper function
################################################
# Read pressure (multiple pressures later?) from ESP32 serial, return to queue 'q'
def read_pressure(q, serial_port, baud_rate=esp_baud):
    raw = []
    timestamp = []
    init_time = time.time()
    loop_count = 0
    try:
        with serial.Serial(port=serial_port, baudrate=baud_rate, bytesize=8) as ser:
            while(valve_rec_bool):
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline()
                    except:
                        continue
                    # if line:
                    raw.append(line)
                    timestamp.append(int((time.time()-init_time)*1000))
                loop_count += 1
    except:
        print(f'Unable to read Serial Input.')
        valve_error_bool = True
        return

    pressure_vals = []
    for row in raw:
        try:
            line = row.decode('utf-8').rstrip()
        except:
            pressure_vals.append(-1)
            continue
        try:
            value = parse_string(line, float_only=True)[0]
            pressure_vals.append(value)
        except:
            value = parse_string(line, float_only=True)
            if len(value):
                try:
                    pressure_vals.append(value)
                except:
                    pressure_vals.append(-1)
            else:
                pressure_vals.append(-1)
        

    print(f'Posting pressure vals, shape: {len(pressure_vals)}')
    q.put( [timestamp, pressure_vals] )


#################################################
# Valve Experiment - Main Function
#################################################

def Valve_Experiment(in_dict, RUNTIME = 2, in_key = 'test', file_dir = None, file_name = None, save = True):

    print(f'Running pump for {RUNTIME} seconds ...')

    # use a Queue for thread-safe communication
    valve_queue = Queue()

    thread_pressure = threading.Thread(target=read_pressure, args=(valve_queue, esp_serial, esp_baud))

    # Start recording
    global valve_rec_bool 
    global valve_error_bool
    valve_rec_bool = True
    valve_error_bool = False
    # Start thread
    thread_pressure.start()

    # Start the pump
    start_time = time.time()

    send_error = send_character(esp_serial, esp_baud, AIR_PUMP_ON, return_error=True)
    if send_error:
        print(f'Failed communication with Microcontroller. Stopping Script.')
        return

    # Wait for run
    try:
        while(time.time()-start_time < RUNTIME):
            if valve_error_bool:
                print(f'Error occured! Stopping the function.')
                send_character(esp_serial, esp_baud, AIR_PUMP_OFF)  # Ensure the pump is stopped
                valve_rec_bool = False
                if thread_pressure.is_alive():
                    thread_pressure.join()
                return
            else:
                pass
    except KeyboardInterrupt:
        print("Kernel interrupted! Stopping the function.")
        # Perform any cleanup if necessary
        send_character(esp_serial, esp_baud, AIR_PUMP_OFF)  # Ensure the pump is stopped
        valve_rec_bool = False
        if thread_pressure.is_alive():
            thread_pressure.join()

    # Stop the pump
    send_character(esp_serial, esp_baud, AIR_PUMP_OFF)

    # Record the data
    valve_rec_bool = False
    thread_pressure.join()
    input = valve_queue.get()

    # add to existing dictionary
    key_name = in_key
    dict_set = in_dict

    # see if key exists
    if key_name in dict_set.keys():
        print("Key exists, appending")
        dict_set[key_name].append(input)
    else:
        dict_set[key_name] = [input] # save as a list
        print("Key does not exist, creating new key")

    # save if necessary
    if save and file_dir and file_name:
        init_dir = os.getcwd()
        os.chdir(file_dir)
        pickle.dump(dict_set, open(file_name, "wb"))
        os.chdir(init_dir)

    return dict_set


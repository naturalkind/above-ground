import time
import curses
import os
import sys
from multiprocessing import Process, Value, Array, Manager
from collections import deque
from itertools import cycle
import tracker_lib
from yamspy import MSPy

# Max periods for:
CTRL_LOOP_TIME = 1/100
SLOW_MSGS_LOOP_TIME = 1/5 # these messages take a lot of time slowing down the loop...

NO_OF_CYCLES_AVERAGE_GUI_TIME = 10


#
# On Linux, your serial port will probably be something like
# /dev/ttyACM0 or /dev/ttyS0 or the same names with numbers different from 0
#
# On Windows, I would expect it to be 
# COM1 or COM2 or COM3...
#
# This library uses pyserial, so if you have more questions try to check its docs:
# https://pyserial.readthedocs.io/en/latest/shortintro.html
#
#
SERIAL_PORT = "/dev/ttyACM0"


    
CMDS = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 1000,
        'yaw':      1500,
        'aux1':     1000,
        'aux2':     1000
        }

# This order is the important bit: it will depend on how your flight controller is configured.
# Below it is considering the flight controller is set to use AETR.
# The names here don't really matter, they just need to match what is used for the CMDS dictionary.
# In the documentation, iNAV uses CH5, CH6, etc while Betaflight goes aux2, aux3...
CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
start_fly = False
ixx = 0
# "print" doesn't work with curses, use addstr instead
try:
    dict_ = {}
    with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
        if board == 1: 
            print ("OK")

        average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)

        command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                        'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                        'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']

        if board.INAV:
            command_list.append('MSPV2_INAV_ANALOG')
            command_list.append('MSP_VOLTAGE_METER_CONFIG')

        for msg in command_list: 
            if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                dataHandler = board.receive_msg()
                board.process_recv_data(dataHandler)
        if board.INAV:
            cellCount = board.BATTERY_STATE['cellCount']
        else:
            cellCount = 0  
        min_voltage = board.BATTERY_CONFIG['vbatmincellvoltage']*cellCount
        warn_voltage = board.BATTERY_CONFIG['vbatwarningcellvoltage']*cellCount
        max_voltage = board.BATTERY_CONFIG['vbatmaxcellvoltage']*cellCount


        slow_msgs = cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])

        last_loop_time = last_slow_msg_time = last_cycleTime = time.time()
        
        local_fast_read_attitude = board.fast_read_attitude
        local_fast_read_imu = board.fast_read_imu
        local_fast_read_altitude = board.fast_read_altitude
        while True:
            start_time = time.time()

            if ixx > 40:
                CMDS['aux2'] = 1500
                start_fly = True
                
            print (start_fly)
            if start_fly:
                if ixx > 200:
                    if CMDS['throttle'] <= 1680 or CMDS['throttle'] <= 1000:
                        CMDS['throttle'] = CMDS['throttle'] + pid_output_throttle 
            ixx += 1
            time.sleep(0.0001)
            
            
            if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                last_loop_time = time.time()
                if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)
            local_fast_read_imu() 
            local_fast_read_attitude()
            local_fast_read_altitude()
            
            
            
            dict_["current_height"] = board.SENSOR_DATA['altitude']
            dict_["rotation"] = board.SENSOR_DATA['kinematics']
            
              
            if (time.time()-last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
                last_slow_msg_time = time.time()

                next_msg = next(slow_msgs)  

                if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)
                    
                if next_msg == 'MSP_ANALOG':
                    voltage = board.ANALOG['voltage']
                    voltage_msg = ""
                    if min_voltage < voltage <= warn_voltage:
                        voltage_msg = "LOW BATT WARNING"
                    elif voltage <= min_voltage:
                        voltage_msg = "ULTRA LOW BATT!!!"
                    elif voltage >= max_voltage:
                        voltage_msg = "VOLTAGE TOO HIGH"


                elif next_msg == 'MSP_STATUS_EX':
                    ARMED = board.bit_check(board.CONFIG['mode'],0)

                    

            end_time = time.time()
            last_cycleTime = end_time-start_time
            if (end_time-start_time)<CTRL_LOOP_TIME:
                time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                
            average_cycle.append(end_time-start_time)
            average_cycle.popleft()

finally:
    pass
    
    
    
    
#{'apiVersion': '1.45.0', 'flightControllerIdentifier': 'BTFL', 'flightControllerVersion': '4.4.3', 'version': 0, 'buildInfo': 'Nov 14 2023 16:10:10', 'multiType': 0, 'msp_version': 0, 'capability': 0, 'cycleTime': 313, 'i2cError': 0, 'activeSensors': 33, 'mode': 0, 'profile': 0, 'uid': [2883621, 808666896, 943206708], 'accelerometerTrims': [0, 0], 'name': '', 'displayName': 'JOE PILOT', 'numProfiles': 4, 'rateProfile': 0, 'boardType': 2, 'armingDisableCount': 26, 'armingDisableFlags': 0, 'armingDisabled': False, 'runawayTakeoffPreventionDisabled': False, 'boardIdentifier': 'S405', 'boardVersion': 0, 'commCapabilities': 55, 'targetName': 'STM32F405', 'boardName': 'SPEEDYBEEF405V3', 'manufacturerId': 'SPBE', 'signature': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'mcuTypeId': 1, 'mspProtocolVersion': 0, 'cpuload': 18, 'flightModeFlags': []}
#['ADJUSTMENT_RANGES', 'ADVANCED_TUNING', 'ANALOG', 'ARMING_CONFIG', 'AUX_CONFIG', 'AUX_CONFIG_IDS', 'BATTERY_CONFIG', 'BATTERY_STATE', 'BAUD_RATES', 'BEEPER_CONFIG', 'BLACKBOX', 'BOARD_ALIGNMENT_CONFIG', 'COMPASS_CONFIG', 'CONFIG', 'CURRENT_METERS', 'CURRENT_METER_CONFIGS', 'DATAFLASH', 'FAILSAFE_CONFIG', 'FC_CONFIG', 'FEATURE_CONFIG', 'FILTER_CONFIG', 'GPS_CONFIG', 'GPS_DATA', 'GPS_RESCUE', 'INAV', 'JUMBO_FRAME_SIZE_LIMIT', 'MISC', 'MIXER_CONFIG', 'MODE_RANGES', 'MODE_RANGES_EXTRA', 'MOTOR_3D_CONFIG', 'MOTOR_CONFIG', 'MOTOR_DATA', 'MSPCodes', 'MSPCodes2Str', 'PID', 'PIDNAMES', 'PID_ADVANCED_CONFIG', 'PIDs', 'RC', 'RC_DEADBAND_CONFIG', 'RC_MAP', 'RC_TUNING', 'REBOOT_TYPES', 'RSSI_CONFIG', 'RXFAIL_CONFIG', 'RX_CONFIG', 'SDCARD', 'SENSOR_ALIGNMENT', 'SENSOR_CONFIG', 'SENSOR_DATA', 'SERIAL_CONFIG', 'SERIAL_PORT_FUNCTIONS', 'SERVO_CONFIG', 'SERVO_DATA', 'SIGNATURE_LENGTH', 'TRANSPONDER', 'VOLTAGE_METERS', 'VOLTAGE_METER_CONFIGS', '__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__enter__', '__eq__', '__exit__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_crc8_dvb_s2', 'armingDisableFlagNames_BF', 'armingDisableFlagNames_INAV', 'basic_info', 'bit_check', 'conn', 'connect', 'convert', 'dataHandler_init', 'fast_msp_rc_cmd', 'fast_read_altitude', 'fast_read_analog', 'fast_read_attitude', 'fast_read_imu', 'is_serial_open', 'process_MSP2_INAV_DEBUG', 'process_MSP2_PID', 'process_MSPV2_INAV_ANALOG', 'process_MSPV2_INAV_MISC', 'process_MSP_ACC_CALIBRATION', 'process_MSP_ACC_TRIM', 'process_MSP_ADJUSTMENT_RANGES', 'process_MSP_ADVANCED_CONFIG', 'process_MSP_ALTITUDE', 'process_MSP_ANALOG', 'process_MSP_API_VERSION', 'process_MSP_ARMING_CONFIG', 'process_MSP_ARMING_DISABLE', 'process_MSP_ATTITUDE', 'process_MSP_BATTERY_CONFIG', 'process_MSP_BATTERY_STATE', 'process_MSP_BEEPER_CONFIG', 'process_MSP_BLACKBOX_CONFIG', 'process_MSP_BOARD_ALIGNMENT_CONFIG', 'process_MSP_BOARD_INFO', 'process_MSP_BOXIDS', 'process_MSP_BOXNAMES', 'process_MSP_BUILD_INFO', 'process_MSP_CF_SERIAL_CONFIG', 'process_MSP_COMPASS_CONFIG', 'process_MSP_COMP_GPS', 'process_MSP_COPY_PROFILE', 'process_MSP_CURRENT_METERS', 'process_MSP_CURRENT_METER_CONFIG', 'process_MSP_DATAFLASH_ERASE', 'process_MSP_DATAFLASH_SUMMARY', 'process_MSP_DEBUG', 'process_MSP_EEPROM_WRITE', 'process_MSP_FAILSAFE_CONFIG', 'process_MSP_FC_VARIANT', 'process_MSP_FC_VERSION', 'process_MSP_FEATURE_CONFIG', 'process_MSP_FILTER_CONFIG', 'process_MSP_GPSSTATISTICS', 'process_MSP_GPS_CONFIG', 'process_MSP_GPS_RESCUE', 'process_MSP_GPS_SV_INFO', 'process_MSP_LOOP_TIME', 'process_MSP_MAG_CALIBRATION', 'process_MSP_MISC', 'process_MSP_MIXER_CONFIG', 'process_MSP_MODE_RANGES', 'process_MSP_MODE_RANGES_EXTRA', 'process_MSP_MOTOR', 'process_MSP_MOTOR_3D_CONFIG', 'process_MSP_MOTOR_CONFIG', 'process_MSP_NAME', 'process_MSP_OSD_CHAR_READ', 'process_MSP_OSD_CHAR_WRITE', 'process_MSP_OSD_CONFIG', 'process_MSP_PID', 'process_MSP_PIDNAMES', 'process_MSP_PID_ADVANCED', 'process_MSP_PID_CONTROLLER', 'process_MSP_RAW_GPS', 'process_MSP_RAW_IMU', 'process_MSP_RC', 'process_MSP_RC_DEADBAND', 'process_MSP_RC_TUNING', 'process_MSP_RESET_CONF', 'process_MSP_RSSI_CONFIG', 'process_MSP_RXFAIL_CONFIG', 'process_MSP_RX_CONFIG', 'process_MSP_RX_MAP', 'process_MSP_SDCARD_SUMMARY', 'process_MSP_SELECT_SETTING', 'process_MSP_SENSOR_ALIGNMENT', 'process_MSP_SENSOR_CONFIG', 'process_MSP_SERVO', 'process_MSP_SERVO_CONFIGURATIONS', 'process_MSP_SET_ACC_TRIM', 'process_MSP_SET_ADJUSTMENT_RANGE', 'process_MSP_SET_ADVANCED_CONFIG', 'process_MSP_SET_ARMING_CONFIG', 'process_MSP_SET_BEEPER_CONFIG', 'process_MSP_SET_BLACKBOX_CONFIG', 'process_MSP_SET_BOARD_ALIGNMENT_CONFIG', 'process_MSP_SET_CF_SERIAL_CONFIG', 'process_MSP_SET_CURRENT_METER_CONFIG', 'process_MSP_SET_FAILSAFE_CONFIG', 'process_MSP_SET_FEATURE_CONFIG', 'process_MSP_SET_FILTER_CONFIG', 'process_MSP_SET_GPS_CONFIG', 'process_MSP_SET_LOOP_TIME', 'process_MSP_SET_MIXER_CONFIG', 'process_MSP_SET_MODE_RANGE', 'process_MSP_SET_MOTOR', 'process_MSP_SET_MOTOR_3D_CONFIG', 'process_MSP_SET_MOTOR_CONFIG', 'process_MSP_SET_NAME', 'process_MSP_SET_OSD_CONFIG', 'process_MSP_SET_PID', 'process_MSP_SET_PID_ADVANCED', 'process_MSP_SET_PID_CONTROLLER', 'process_MSP_SET_RAW_RC', 'process_MSP_SET_RC_DEADBAND', 'process_MSP_SET_RC_TUNING', 'process_MSP_SET_REBOOT', 'process_MSP_SET_RESET_CURR_PID', 'process_MSP_SET_RSSI_CONFIG', 'process_MSP_SET_RTC', 'process_MSP_SET_RXFAIL_CONFIG', 'process_MSP_SET_RX_CONFIG', 'process_MSP_SET_RX_MAP', 'process_MSP_SET_SENSOR_ALIGNMENT', 'process_MSP_SET_SENSOR_CONFIG', 'process_MSP_SET_SERVO_CONFIGURATION', 'process_MSP_SET_TRANSPONDER_CONFIG', 'process_MSP_SET_VOLTAGE_METER_CONFIG', 'process_MSP_SET_VTX_CONFIG', 'process_MSP_SONAR', 'process_MSP_STATUS', 'process_MSP_STATUS_EX', 'process_MSP_UID', 'process_MSP_VOLTAGE_METERS', 'process_MSP_VOLTAGE_METER_CONFIG', 'process_MSP_VTX_CONFIG', 'process_armingDisableFlags', 'process_mode', 'process_recv_data', 'readbytes', 'reboot', 'receive_msg', 'receive_raw_msg', 'save2eprom', 'send_RAW_MOTORS', 'send_RAW_RC', 'send_RAW_msg', 'ser_trials', 'serialPortFunctionMaskToFunctions', 'serial_port_read_lock', 'serial_port_write_lock', 'set_ARMING_DISABLE', 'set_FEATURE_CONFIG', 'set_RX_MAP']
    
    
    
    
    
    
    
    
    
    
    
    
    
    

#!/usr/bin/python
# Sparki ROS Node by Brad Hayes for CSCI 3302 at University of Colorado Boulder -- A work in progress (2019)
# Serial code and Sparki INO file heavily borrowed/adapted from the Sparki Myro repo by Jeremy Eglen

from __future__ import division
from __future__ import print_function

import datetime
import logging
import math
import os
import platform
import time
import sys
import serial
import rospy
import json
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16, String, Float32, Float32MultiArray, Empty

TERMINATOR = chr(23)  # this character is at the end of every message to / from Sparki
SYNC = chr(22)  # this character is sent by Sparki after every command completes so we know it's ready for the next
MAX_TRANSMISSION = 20  # maximum message length is 20 to conserve Sparki's limited RAM

LCD_BLACK = 0  # set in Sparki.h
LCD_WHITE = 1  # set in Sparki.h

DEBUG_DEBUG = 5  # reports just about everything
DEBUG_INFO = 4  # reports entering functions
DEBUG_WARN = 3  # a generally sane default; reports issues that may be mistakes, but don't interfere with operation
DEBUG_ERROR = 2  # reports something contrary to the API
DEBUG_CRITICAL = 1  # reports an error which interferes with proper or consistent operation
DEBUG_ALWAYS = 0  # should always be reported

CONN_TIMEOUT = 0.1  # in seconds

# ***** COMMAND CHARACTER CODES ***** #
# This is the list of possible command codes; note that it is possible for some commands to be turned off at Sparki's level (e.g. the Accel, Mag)
COMMAND_CODES = {
    'BEEP': 'b',  # requires 2 arguments: int freq and int time; returns nothing
    'COMPASS': 'c',  # no arguments; returns float heading
    'GAMEPAD': 'e',  # no arguments; returns nothing
    'GET_ACCEL': 'f',  # no arguments; returns array of 3 floats with values of x, y, and z
    # 'GET_BATTERY': 'j',  # no arguments; returns float of voltage remaining
    'GET_LIGHT': 'k',  # no arguments; returns array of 3 ints with values of left, center & right light sensor
    'GET_LINE': 'm',
    # no arguments; returns array of 5 ints with values of left edge, left, center, right & right edge line sensor
    'GET_MAG': 'o',  # no arguments; returns array of 3 floats with values of x, y, and z
    'GRIPPER_CLOSE_DIS': 'v',  # requires 1 argument: float distance to close the gripper; returns nothing
    'GRIPPER_OPEN_DIS': 'x',  # requires 1 argument: float distance to open the gripper; returns nothing
    'GRIPPER_STOP': 'y',  # no arguments; returns nothing
    'INIT': 'z',  # no arguments; confirms communication between computer and robot
    'LCD_CLEAR': '0',  # no arguments; returns nothing
    ## below LCD commands removed for compacting purposes
    ##'LCD_DRAW_CIRCLE':'1',    # requires 4 arguments: int x&y, int radius, and int filled (1 is filled); returns nothing
    ##'LCD_DRAW_LINE': '2',
    # requires 4 arguments ints x&y for start point and x1&y1 for end points; returns nothing; EXT_LCD_1 must be True
    'LCD_DRAW_PIXEL': '3',  # requires 2 arguments: int x&y; returns nothing
    ##'LCD_DRAW_RECT':'4',# requires 5 arguments: int x&y for start point, ints width & height, and int filled (1 is filled); returns nothing
    'LCD_DRAW_STRING': '5',
    # requires 3 arguments: int x (column), int line_number, and char* string; returns nothing; EXT_LCD_1 must be True
    'LCD_PRINT': '6',  # requires 1 argument: char* string; returns nothing
    'LCD_PRINTLN': '7',  # requires 1 argument: char* string; returns nothing
    'LCD_READ_PIXEL': '8',
    # requires 2 arguments: int x&y; returns int color of pixel at that point; EXT_LCD_1 must be True
    'LCD_SET_COLOR': 'T',  # requires 1 argument: int color; returns nothing; EXT_LCD_1 must be True
    'LCD_UPDATE': '9',  # no arguments; returns nothing
    'MOTORS': 'A',  # requires 3 arguments: int left_speed (1-100), int right_speed (1-100), & float time
    # if time < 0, motors will begin immediately and will not stop; returns nothing
    'BACKWARD_CM': 'B',  # requires 1 argument: float cm to move backward; returns nothing
    'FORWARD_CM': 'C',  # requires 1 argument: float cm to move forward; returns nothing
    'PING': 'D',  # no arguments; returns ping at current servo position
    'RECEIVE_IR': 'E',  # no arguments; returns an int read from the IR sensor
    'SEND_IR': 'F',  # requires 1 argument: int to send on the IR sender; returns nothing
    'SERVO': 'G',  # requires 1 argument: int servo position; returns nothing
    'SET_DEBUG_LEVEL': 'H',  # requires 1 argument: int debug level (0-5); returns nothing; SPARKI_DEBUGS must be True
    'SET_RGB_LED': 'I',  # requires 3 arguments: int red, int green, int blue; returns nothing
    'SET_STATUS_LED': 'J',  # requires 1 argument: int brightness of LED; returns nothing
    'STOP': 'K',  # no arguments; returns nothing
    'TURN_BY': 'L',  # requires 1 argument: float degrees to turn - if degrees is positive, turn clockwise,
    # if degrees is negative, turn counterclockwise; returns nothing
    'GET_NAME': 'O',  # get the Sparki's name as stored in the EEPROM - USE_EEPROM must be True
    # if the name was not set previously, can give undefined behavior
    'SET_NAME': 'P',  # set the Sparki's name in the EEPROM - USE_EEPROM must be True
    'READ_EEPROM': 'Q',  # reads data as stored in the EEPROM - USE_EEPROM & EXT_LCD_1 must be True
    'WRITE_EEPROM': 'R',  # writes data to the EEPROM - USE_EEPROM & EXT_LCD_1 must be True
    'NOOP': 'Z'  # does nothing and returns nothing - NOOP must be True
}

# ***** SENSOR POSITION CONSTANTS ***** #
# LINE SENSORS #
LINE_EDGE_RIGHT = 4
LINE_MID_RIGHT = 3
LINE_MID = 2
LINE_MID_LEFT = 1
LINE_EDGE_LEFT = 0

# ***** MAX GRIPPER DISTANCE ***** #
MAX_GRIPPER_DISTANCE = 7.0

# ***** SERVO POSITIONS ***** #
SERVO_LEFT = -80
SERVO_CENTER = 0
SERVO_RIGHT = 80

# ***** TIMING VARIABLES ***** #
LAST_FINISH_TIME = None
DELAY_CONST = None
init_time = None

serial_port = None  # save the serial port on which we're connected
serial_conn = None  # hold the pyserial object
serial_is_connected = False  # set to true once connection is done

odometry_x, odometry_y, odometry_theta = 0., 0., 0.
sparki_servo_theta = 0.

motor_speed_left, motor_speed_right = 0., 0.  # Percentage of max speed, [-100, 100]
SPARKI_LINEAR_SPEED = 0.0278  # 100% speed in m/s
SPARKI_ANGULAR_SPEED = 360.0 / 9.605  # angular velocity in degrees / second
SPARKI_AXLE_DIAMETER = 0.085  # Distance between wheels, meters
SPARKI_WHEEL_RADIUS = 0.03  # Radius of wheels, meters
CYCLE_TIME = 0.05  # Minimum delay between cycles
IR_CYCLE_TIME = 0.05  # Minimum Delay polling IR sensors
LAST_IR_POLL = 0
sparki_ir_sensors = [0, 0, 0, 0, 0]
pub_sparki_odom, pub_sparki_state = None, None
sparki_ping_requested = False


def main(com_port):
    global pub_sparki_odom, pub_sparki_state, serial_conn
    rospy.init_node("sparki_driver_ros", log_level=rospy.DEBUG)
    init(com_port)

    pub_sparki_odom = rospy.Publisher('/sparki/odometry', Pose2D, queue_size=10)
    pub_sparki_state = rospy.Publisher('/sparki/state', String, queue_size=10)

    # CUSTOM
    sub_sparki_turn = rospy.Subscriber('/sparki/turn_command', Float32, send_turn_command)
    sub_sparki_forward = rospy.Subscriber('/sparki/forward_command', Float32, send_move_forward)

    sub_sparki_motors = rospy.Subscriber('/sparki/motor_command', Float32MultiArray, send_motor_command)
    sub_sparki_ping = rospy.Subscriber('/sparki/ping_command', Empty, send_ping)
    sub_sparki_odom = rospy.Subscriber('/sparki/set_odometry', Pose2D, set_odometry)
    sub_sparki_servo = rospy.Subscriber('/sparki/set_servo', Int16, set_servo)

    last_time = time.time()
    while not rospy.is_shutdown():
        try:
            cycle_start = time.time()
            # Update and Publish Odometry
            update_and_publish_odometry(pub_sparki_odom, time.time() - last_time)
            last_time = time.time()
            # Update and Publish Sensors
            update_and_publish_state(pub_sparki_state)
            rospy.sleep(max(0, CYCLE_TIME - (time.time() - cycle_start)))
        except serial.SerialException:
            rospy.loginfo("Serial port reset for some reason! Reconnecting.")
            init(com_port)


def set_odometry(data):
    global odometry_x, odometry_y, odometry_theta

    odometry_x, odometry_y, odometry_theta = data.x, data.y, data.theta


def set_servo(data):
    global sparki_servo_theta
    servo(-data.data)  # Sparki's rotation axis is the reverse of ours, so negate
    sparki_servo_theta = math.radians(data.data)


# Custom command to make wheels turn!
def send_turn_command(data):
    global LAST_FINISH_TIME
    if isinstance(data, Float32) is False:
        print('Invalid turn command received')
        print(str(data))
        return

    printDebug('Turn Request Received', DEBUG_INFO)
    sendSerial(COMMAND_CODES['TURN_BY'], [data.data])

    LAST_FINISH_TIME = rospy.Time.now() + rospy.Duration.from_sec(abs(data.data) / SPARKI_ANGULAR_SPEED)


def send_move_forward(data):
    global LAST_FINISH_TIME
    if isinstance(data, Float32) is False:
        print('Invalid move forward command received')
        print(str(data))
        return
    printDebug('Move Forward Request Received', DEBUG_INFO)

    dist_in_cm = data.data * 100.0
    direction = 'FORWARD_CM' if dist_in_cm > 0 else 'BACKWARD_CM'
    dist_in_cm = abs(dist_in_cm)

    sendSerial(COMMAND_CODES[direction], [dist_in_cm])

    LAST_FINISH_TIME = rospy.Time.now() + rospy.Duration.from_sec(abs(data.data) / SPARKI_LINEAR_SPEED)


def send_motor_command(data):
    if len(data.data) != 2:
        print("Invalid motor command received")
        print(str(data))
        return
    left_motor_speed = data.data[0]
    right_motor_speed = data.data[1]

    motors(left_motor_speed, right_motor_speed)


def send_ping(data):
    global sparki_ping_requested
    sparki_ping_requested = True


def update_and_publish_state(pub):
    global sparki_servo_theta, sparki_ir_sensors, LAST_IR_POLL, sparki_ping_requested
    state = {}
    state['servo'] = sparki_servo_theta

    if (time.time() - LAST_IR_POLL > IR_CYCLE_TIME):
        sparki_ir_sensors = getLine()
        LAST_IR_POLL = time.time()

    state['light_sensors'] = sparki_ir_sensors

    if sparki_ping_requested is True:
        sparki_ping_dist = ping()
        state['ping'] = sparki_ping_dist
        sparki_ping_requested = False

    pub.publish(json.dumps(state))


def update_and_publish_odometry(pub, time_delta):
    global SPARKI_LINEAR_SPEED, SPARKI_AXLE_DIAMETER
    global motor_speed_left, motor_speed_right
    global odometry_x, odometry_y, odometry_theta
    left_wheel_dist = (motor_speed_left * time_delta * SPARKI_LINEAR_SPEED)
    right_wheel_dist = (motor_speed_right * time_delta * SPARKI_LINEAR_SPEED)

    odometry_x += math.cos(odometry_theta) * (left_wheel_dist + right_wheel_dist) / 2.
    odometry_y += math.sin(odometry_theta) * (left_wheel_dist + right_wheel_dist) / 2.
    odometry_theta += (right_wheel_dist - left_wheel_dist) / SPARKI_AXLE_DIAMETER

    pose = Pose2D()
    pose.x, pose.y, pose.theta = odometry_x, odometry_y, odometry_theta
    pub.publish(pose)


# -------------------------------------

def printDebug(message, priority=logging.WARN):
    """ Logs message given the priority specified 
    
        arguments:
        message - the string message to be logged
        priority - the integer priority of the message; uses the priority levels in the logging module

        returns:
        nothing
    """

    # for compatibility, we will recognize the "old" priority levels, but new code should be written to conform to the
    # priority levels in the logging module
    if priority == DEBUG_DEBUG or priority == logging.DEBUG:
        rospy.logdebug(message)
    elif priority == DEBUG_INFO or priority == logging.INFO:
        rospy.loginfo(message)
    elif priority == DEBUG_WARN or priority == logging.WARN:
        rospy.logwarn(message)
    elif priority == DEBUG_ERROR or priority == logging.ERROR:
        rospy.logerr(message)
    elif priority == DEBUG_CRITICAL or priority == logging.CRITICAL:
        rospy.logfatal(message)
    else:
        print("[{}] --- {}".format(time.ctime(), message), file=sys.stderr)


def motors(left_speed, right_speed):
    """ Moves wheels at left_speed and right_speed % of their max speed    
        arguments:
        left_speed - the left wheel speed; a float between -1.0 and 1.0
        right_speed - the right wheel speed; a float between -1.0 and 1.0
        
        returns:
        nothing
    """
    global motor_speed_left, motor_speed_right

    printDebug("Motor Request Received", DEBUG_INFO)

    if left_speed == 0 and right_speed == 0:
        sendSerial(COMMAND_CODES["STOP"])
        motor_speed_left = 0.
        motor_speed_right = 0.
        return

    if left_speed < -1.0 or left_speed > 1.0:
        printDebug("In motors, left_speed is outside of the range -1.0 to 1.0", DEBUG_ERROR)

    if right_speed < -1.0 or right_speed > 1.0:
        printDebug("In motors, right_speed is outside of the range -1.0 to 1.0", DEBUG_ERROR)

    # adjust speeds to Sparki's requirements
    left_speed = min(max(left_speed, -1.0), 1.0)
    right_speed = min(max(right_speed, -1.0), 1.0)

    left_speed = int(left_speed * 100)  # sparki expects an int between 1 and 100
    right_speed = int(right_speed * 100)  # sparki expects an int between 1 and 100
    args = [left_speed, right_speed]

    sendSerial(COMMAND_CODES["MOTORS"], args)
    motor_speed_left = left_speed / 100.
    motor_speed_right = right_speed / 100.


def ping():
    """ Returns the reading from the ultrasonic sensor on the servo

        returns:
        int - approximate distance in centimeters from nearest object (-1 means nothing was found)
    """
    printDebug("Ping Request Received", DEBUG_INFO)

    sendSerial(COMMAND_CODES["PING"])
    result = getSerialInt()
    return result


def servo(deg):
    """ Sets the servo angle [-80,80]

        returns: nothing
    """
    printDebug("Servo Request Received: %d" % deg, DEBUG_INFO)

    sendSerial(COMMAND_CODES["SERVO"], [int(max(min(80, deg), -80))])


def getLine():
    """ Returns the value of the line sensor: [LINE_EDGE_LEFT, LINE_MID_LEFT, LINE_MID, LINE_MID_RIGHT, LINE_EDGE_RIGHT]
                
        returns:
        tuple of ints - values of edge left, left, middle, right, and edge right sensors (in that order)
    """
    sendSerial(COMMAND_CODES["GET_LINE"])
    lines = (getSerialInt(), getSerialInt(), getSerialInt(), getSerialInt(), getSerialInt())

    return lines


def init(com_port, print_versions=True):
    """ Connects to the Sparki robot on com_port; if it is already connected, this will disconnect and reconnect on the given port
        Note that Sparki MUST already be paired with the computer over Bluetooth
        
        arguments:
        com_port - a string designating which port Sparki is on (windows looks like "COM??"; mac and linux look like "/dev/????"
                   if com_port is the string "mac", this will assume the standard mac port ("/dev/tty.ArcBotics-DevB")
                   if com_port is the string "hc06", this will assume the standard HC-06 port ("tty.HC-06-DevB")
        print_versions - boolean whether or not to print connection message
        
        returns:
        nothing
    """
    global init_time
    global serial_conn
    global serial_port
    global serial_is_connected
    global DELAY_CONST

    DELAY_CONST = rospy.Duration.from_sec(0.005)  # 5ms delay

    printDebug("In init, com_port is " + str(com_port), DEBUG_INFO)

    if serial_is_connected:
        disconnectSerial()

    if com_port == "mac":
        com_port = "/dev/tty.ArcBotics-DevB"
    elif com_port == "hc06":
        com_port = "/dev/tty.HC-06-DevB"

    serial_port = com_port

    try:
        serial_conn = serial.Serial(port=serial_port, baudrate=9600, timeout=CONN_TIMEOUT)
    except serial.SerialException:
        printDebug("Unable to Connect over Serial to " + serial_port, DEBUG_ERROR)
        raise

    serial_is_connected = True  # have to do this prior to sendSerial, or sendSerial will never try to send

    sendSerial(COMMAND_CODES["INIT"])
    init_message = getSerialString()

    if init_message:
        init_time = time.time()

        printDebug("Sparki connection successful", DEBUG_ALWAYS)
    else:
        printDebug("Sparki communication failed", DEBUG_ALWAYS)
        serial_is_connected = False
        init_time = -1


def disconnectSerial():
    """ Disconnects from the Sparki robot
    """
    global init_time
    global serial_conn
    global serial_is_connected
    global serial_port

    if serial_is_connected:
        serial_is_connected = False
        serial_conn.close()
        serial_conn = None
        serial_port = None
        init_time = -1


def getSerialBytes():
    """ Returns bytes from the serial port up to TERMINATOR
    
        arguments:
        none
        
        returns:
        string - created from bytes in the serial port
    """
    global serial_conn
    global serial_is_connected

    if not serial_is_connected:
        printDebug("Sparki is not connected - use init()", DEBUG_CRITICAL)
        raise RuntimeError

    result = bytearray()

    try:
        inByte = serial_conn.read()
    except serial.SerialTimeoutException:
        printDebug("Error communicating with Sparki", DEBUG_CRITICAL)
        raise

    printDebug("Getting Bytes... first byte is " + str(inByte), DEBUG_DEBUG)

    while inByte != TERMINATOR.encode():  # read until we see a TERMINATOR
        if inByte != SYNC.encode():  # ignore it - we don't care about SYNCs
            result = result + inByte

        try:
            inByte = serial_conn.read()
        except serial.SerialTimeoutException:
            printDebug("Error communicating with Sparki", DEBUG_CRITICAL)
            raise

        if inByte != SYNC.encode():
            printDebug("Next byte is '" + str(inByte) + "'", DEBUG_DEBUG)

    printDebug("Finished fetching bytes, result is '" + str(result) + "'", DEBUG_DEBUG)
    return result.decode()


def getSerialChar():
    """ Returns the next char (str) from the serial port
    
        arguments:
        none
        
        returns:
        char (str) - from the serial port
    """
    result = chr(int(getSerialBytes()))

    printDebug("In getSerialChar, returning " + result, DEBUG_DEBUG)
    return result


def getSerialFloat():
    """ Returns the next float from the serial port
    
        arguments:
        none
        
        returns:
        float - from the serial port; returns a -1 if Sparki gave "ovf" or no response
    """
    result = getSerialBytes()

    if str(result) == "ovf" or len(result) == 0:  # check for overflow
        result = -1.0  # -1.0 is not necessarily a great "error response", except that values from the Sparki should be positive
    else:
        result = float(result)

    printDebug("In getSerialFloat, returning " + str(result), DEBUG_DEBUG)
    return result


def getSerialInt():
    """ Returns the next int from the serial port
    
        arguments:
        none
        
        returns:
        int - from the serial port; returns a -1 if Sparki gave "ovf" or no response
    """
    result = getSerialBytes()

    if str(result) == "ovf" or len(result) == 0:  # check for overflow
        result = -1  # -1 is not necessarily a great "error response", except that values from the Sparki should be positive
    else:
        result = int(result)

    printDebug("In getSerialInt, returning " + str(result), DEBUG_DEBUG)
    return result


def getSerialString():
    """ Returns the next string from the serial port
    
        arguments:
        none
        
        returns:
        string - from the serial port
    """
    result = str(getSerialBytes())

    printDebug("In getSerialString, returning " + result, DEBUG_DEBUG)
    return result


def sendSerial(command, args=None):
    """ Sends the command with the args over a serial connection
        
        arguments:
        command - a character command code as defined at the top of this file
        args - a list of arguments to be sent; optional
        
        returns:
        nothing
    """
    global serial_conn
    global serial_is_connected
    global serial_port

    if not serial_is_connected:
        printDebug("In sendSerial, Sparki is not connected - use init()", DEBUG_ALWAYS)
        raise RuntimeError("Attempt to send message to Sparki without initialization")

    if command is None:
        printDebug("In sendSerial, no command given", DEBUG_ALWAYS)
        raise RuntimeError("Attempt to send message to Sparki without command")

    # command_queue.append((command, args))  # keep track of every command sent

    try:
        waitForSync()  # be sure Sparki is available before sending
    except serial.SerialTimeoutException:  # Macs seem to be sensitive to disconnecting, so we try to reconnect if we have a problem
        # if there's a failure, try to reconnect unless we're init'ing
        if command != COMMAND_CODES["INIT"]:
            init(serial_port, False)
            try:
                waitForSync()
            except:
                printDebug("In sendSerial, retry failed", DEBUG_CRITICAL)
                raise
        else:
            printDebug("In sendSerial, serial timeout on init", DEBUG_CRITICAL)
            raise

    printDebug("In sendSerial, Sending command - " + command, DEBUG_DEBUG)

    values = []  # this will hold what we're actually sending to Sparki

    values.append(command)

    if args is not None:
        if isinstance(args, str):
            values.append(args)
            printDebug("In sendSerial, values is " + str(values), DEBUG_DEBUG)
        else:
            values = values + args

    for value in values:
        message = (str(value) + TERMINATOR).encode()

        if len(message) > MAX_TRANSMISSION:
            printDebug("In sendSerial, messages must be " + str(MAX_TRANSMISSION) + " characters or fewer", DEBUG_ERROR)

            # FIXME: The "stop" function does not exist and I'm not entirely sure what it should do
            stop()  # done for safety -- in case robot is in motion

            raise RuntimeError("Messages sent to Sparki must be {} or fewer characters".format(str(MAX_TRANSMISSION)))

        printDebug("Sending bytes (" + str(value) + ")", DEBUG_DEBUG)

        try:
            serial_conn.write(message)
        except serial.SerialTimeoutException:
            printDebug("In sendSerial, error communicating with Sparki", DEBUG_CRITICAL)
            raise

    serial_conn.flush()  # ensure the buffer is flushed
    time.sleep(0.01)


def waitForSync():
    """ Waits for the SYNC character from Sparki

        arguments:
        none

        returns:
        nothing
    """
    global serial_conn
    global serial_is_connected
    global LAST_FINISH_TIME

    start_time = rospy.Time.now()

    # Ensure we have waited long enough for the last blocking command to run
    if LAST_FINISH_TIME is not None and start_time < LAST_FINISH_TIME:
        dur = (LAST_FINISH_TIME - start_time) + DELAY_CONST  # adding constant delay
        printDebug("Waiting {:.2f}s for blocking command to finish".format(dur.to_sec()), DEBUG_INFO)

        rospy.sleep(dur)
        printDebug('Done waiting', DEBUG_INFO)

    LAST_FINISH_TIME = None

    if not serial_is_connected:
        printDebug("Sparki is not connected - use init()", DEBUG_CRITICAL)
        raise RuntimeError("Attempt to listen for message from Sparki without initialization")

    serial_conn.flushInput()  # get rid of any waiting bytes

    start_time = time.time()

    inByte = -1
    loop_wait = 0
    if platform.system() == "Darwin":  # Macs seem to be extremely likely to timeout -- this is attempting to deal with that quickly
        retries = 1  # the number of times to retry connecting in the case of a timeout
        loop_wait = 0  # pause this long each time through the loop
    else:
        retries = 5  # the number of times to retry connecting in the case of a timeout
        loop_wait = .01  # pause this long each time through the loop

    while inByte != SYNC.encode():  # loop, doing nothing substantive, while we wait for SYNC
        if time.time() > start_time + (CONN_TIMEOUT * retries):
            if platform.system() == "Darwin":  # Macs seem to be extremely likely to timeout -- so we report at a different debug level
                printDebug(
                    "In waitForSync, unable to sync with Sparki (this may not be due to power saving settings)",
                    DEBUG_INFO)
            else:
                printDebug("In waitForSync, unable to sync with Sparki", DEBUG_ERROR)
            raise serial.SerialTimeoutException(
                "Unable to sync with Sparki -- may be temporary error due to power saving")

        try:
            inByte = serial_conn.read()
        except serial.SerialTimeoutException:
            printDebug("SerialTimeoutException caught in waitForSync, unable to sync with Sparki", DEBUG_ERROR)
            raise

        time.sleep(loop_wait)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Sparki-ros must be run with a Sparki COM port specified")
        exit()
    main(sys.argv[1])

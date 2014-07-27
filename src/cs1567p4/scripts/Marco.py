#!/usr/bin/python
import rospy
import math
import subprocess
from cs1567p4.srv import *
from std_srvs.srv import * 
from nav_msgs.msg import *
from cs1567p4/msg import *
from geometry_msgs.msg import *
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion

## Constants
LINEAR_SPEED      = 0.15 #TODO determine if good speed for marco, compared to polo
ANGULAR_SPEED     = 0.45 #TODO determine if good speed for marco, compared to polo
LINEAR_THRESHOLD  = 0.05 #TODO determine if good number
ANGULAR_THRESHOLD = 0.05 #TODO determine if good number
BACKUP_DISTANCE   = 5.00 #TODO determine if good number

## Globals
bumper_event    = None
const_cmd_srv   = None
jump_play       = None
send_command    = rospy.ServiceProxy('constant_command', ConstantCommand)
startPosition   = {'x' : 0.0, 'y' : 0.0}
ANGLE           = 0.0
DISTANCE        = 0.0
GAMEOVER        = False
STATE           = ''
LOCATIONS            = [] #array of dictionaries!

##
##
## State Machine Description
##
##
'''
=================  STATE MACHINE  ======================

-> needToCallMarco            : callMarco()
    -> l_getLocation            : locationCallback()

-> needToCalculate            : calculateAndSet()

-> needToTurnDesired          : turnDesired()
    -> o_turnDesired            : odometryCallback()

-> needToTravelDesired        : travelDesired()
    -> o_saveStartPosition      : odometryCallback()
        -> o_travelDesired          : odometryCallback()

=====================  REPEAT  =========================
'''

##
##
## State Machine Functions
##
##
''' 
=== Main action Loop state machine, sets four main commands in motion
'''
def loop():
    if   STATE == 'needToCallMarco':
        callMarco()
    elif STATE == 'needToCalculate':
        calculateAndSet()
    elif STATE == 'needToTurnDesired':
        turnDesired()
    elif STATE == 'needToTravelDesired':
        travelDesired()

''' 
=== Communicate with Jump, get desiredAngle and desiredDistance to closest polo
'''
def callMarco():
    global ANGLE
    global DISTANCE
    global STATE
    # Say Marco
    sayMarco()
    # Everyone stop
    stop()
    jump_StopAll()
    # Look at location list
    STATE = 'l_getLocation'

''' 
=== Calculate closest, set globals
'''
def calculateAndSet():
    global STATE
    getClosestAndSetDistanceAndAngle()
    jump_StartAll()
    STATE = 'needToTurnDesired'

'''
=== Turn left or right (depending) until odometryCallback stops it
'''
def turnDesired():
    global STATE
    command = Twist()
    command.angular.z = ANGULAR_SPEED
    send_command(command)
    STATE = 'o_turnDesired'

''' 
=== Travel forward until odometryCallback stops it
'''
def travelDesired():
    global STATE
    command = Twist()
    command.linear.x = LINEAR_SPEED
    send_command(command)
    STATE = 'o_saveStartPosition'

##
##
## Helper Functions
##
##
''' 
=== Say 'Marco'
'''
def sayMarco():
    espeak = subprocess.Popen(('espeak','marco', '--stdout'),stdout=subprocess.PIPE)
    subprocess.check_output(('paplay'), stdin=espeak.stdout)
    espeak.wait()

''' 
=== Return the currentYaw in radians
'''
def getCurrentYaw(data):
    euler = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    return euler[2] # Radians

''' 
=== Set the global startPosition dictionary
'''
def setStartPosition(data):
    global startPosition
    startPosition['x'] = data.pose.pose.position.x
    startPosition['y'] = data.pose.pose.position.y

''' 
=== Return the currentPosition in a dictionary
'''
def getCurrentPosition(data):
    currentPosition = {}
    currentPosition['x'] = data.pose.pose.position.x
    currentPosition['y'] = data.pose.pose.position.y
    return currentPosition

''' 
=== Calculate the distance between two "Points" (dictionaries with x and y keys)
'''
def calculateDistance(point1, point2):
    x1 = point1['x']
    x2 = point2['x']
    y1 = point1['y']
    y2 = point2['y']
    result = math.sqrt( abs(x1-x2)**2 + abs(y1-y2)**2 ) # '**' is exponentiation operator (**2 == squared)
    return result

''' 
=== Stop all motion
'''
def stop():
    command = Twist()
    command.linear.x = 0.0
    command.angular.z = 0.0
    send_command(command)

''' 
=== Backup after a BumperEvent
'''
def bumperBackup():
    command = Twist()
    command.linear.x = -LINEAR_SPEED
    send_command(command)
    rospy.sleep(BACKUP_DISTANCE)
    stop()

''' 
=== Get closest polo and set DISTANCE -- CALLS SETANGLE AS WELL SORRY
'''
def getClosestAndSetDistanceAndAngle():
    global DISTANCE
    # Set my location
    '''myLocationX = 
    myLocationY = 
    # Set location of polo 1
    polo1LocationX =
    polo1LocationY =
    # Set location of polo 2
    polo2LocationX = 
    polo2LocationY = 
    '''
    distanceTo1 = math.hypot(myLocationX-polo1LocationX, myLocationY-polo1LocationY)
    distanceTo2 = math.hypot(myLocationX-polo2LocationX, myLocationY-polo2LocationY)

    if distanceTo1 > distanceTo2:
        DISTANCE = distanceTo2
        setAngle(myLocationX, myLocationY, polo2LocationX, polo2LocationY)
    else:
        DISTANCE = distanceTo1
        setAngle(myLocationX, myLocationY, polo1LocationX, polo1LocationY)

''' 
=== Set ANGLE based on two locations and quadrant of polo
'''
def setAngle(marcoX, marcoY, poloX, poloY):
    global ANGLE
    ANGLE = math.atan2(marcoY-poloY, marcoX-poloX)
    if (poloX >= 0 and poloY >=0):
        # Q1
        # just angle
        ANGLE = 1 * ANGLE
    elif (poloX >=0 and poloY < 0):
        # Q4
        # negative angle
        ANGLE = -1 * ANGLE
    elif (poloX < 0 and poloY >=0):
        # Q2
        # +90 + angle
        ANGLE = (math.pi/2) + ANGLE
    else: #(poloX<0 and poloY<0)
        # Q3
        # -90 - angle
        ANGLE = -(math.pi/2) - ANGLE

##
##
## Jump Functions
##
##
def jump_StopAll():
    global jump_play
    jump_play.publish(False)

def jump_StartAll():
    global jump_play
    jump_play.publish(True)

##
##
## Callback Functions
##
##
''' 
=== Bumper Callback for a BumperEvent
'''
def bumperCallback(data): #TODO will this work?
    global STATE

    if(data.state == 0): 
        return #return if state is released
    
    STATE = 'interrupted' #dummy state
    stop()
    bumperBackup()
    STATE = 'needToCallMarco'

''' 
=== Location Callback 
'''
def locationCallback(msg):
    global STATE
    if STATE == 'l_getLocation':
        print "LOCATIONLIST"
        print msg
        STATE = 'needToCalculate'

''' 
=== Odometry Callback state machine, determines when to stop the robot and sets next state
'''
def odometryCallback(data):
    global STATE

    if STATE == 'interrupted':
        return

    if STATE == 'o_turnDesired':
        # Get current yaw in radians
        currentYaw = getCurrentYaw(data)
        # Get to desiredAngle w/in threshold
        if abs(currentYaw - ANGLE) <= ANGULAR_THRESHOLD:
            stop()
            STATE = 'needToTravelDesired'

    elif STATE == 'o_saveStartPosition':
        setStartPosition(data)
        STATE = 'o_travelDesired'

    elif STATE == 'o_travelDesired':
        # Get current distance travelled
        distanceTravelled = calculateDistance(startPosition, getCurrentPosition(data)) #global startPosition
        # Get to desiredDistance w/in threshold
        if abs(distanceTravelled - DISTANCE) <= LINEAR_THRESHOLD:
            stop()
            STATE = 'needToCallMarco'


##
##
## Init/Main
##
##
''' 
=== Initialize 
'''
def initialize_commands():
    global const_cmd_srv 
    global bumper_event 
    global jump_play
    global location_list
    rospy.Subscriber('/odom', Odometry, odometryCallback)
    rospy.init_node('marconode', anonymous=True)
    rospy.wait_for_service('constant_command')
    const_cmd_srv = rospy.ServiceProxy('constant_command', ConstantCommand)
    bumper_event = rospy.Subscriber('/mobile_base/events/bumper',BumperEvent, bumperCallback)
    #jump_play = rospy.Publisher('/marco/play', Running)
    location_list = rospy.Subscriber('/tomservo/location', LocationList, locationCallback)

''' 
=== Main
'''
if __name__ == "__main__":   
    try: 
        initialize_commands()

        STATE = 'needToCallMarco' # init -- start facing north!

        while not GAMEOVER:
            poop = 'poop'
            #loop()

    except rospy.ROSInterruptException: pass




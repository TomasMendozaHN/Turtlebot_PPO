import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import time

msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""

# Get odom information
def get_odom():
    try: 
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)

    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return (Point(*trans), rotation[2])



def moveDiscrete():
    global move_cmd
    global cmd_vel
    
    (position, rotation) = get_odom()
    init_x = position.x
    init_y = position.y
    
    distance = 0.2  # meters ==> 40cm
    movedDist = 0
    
    # As long as I am moer than 5cm away, keep moving!
    while (distance - movedDist) > 0.05:   
        (position, _) = get_odom()   # Updat eodometry

        # Calculate euclidean distance
        movedDist = sqrt(pow(init_x - position.x, 2) + pow(init_y - position.y, 2))   
        
        move_cmd.linear.x = 0.15  # Set Speed

        cmd_vel.publish(move_cmd) # Move
        
        r.sleep()  # use rospy sleep function to keep while loop running at desired rate

    rospy.loginfo("Done")

#%% Main
rospy.init_node('ttb3_test', anonymous=False)
cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
position = Point()
move_cmd = Twist()
r = rospy.Rate(2)
tf_listener = tf.TransformListener()
odom_frame = 'odom'


#%% While Loop
while not rospy.is_shutdown():
    try:
        tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
        base_frame = 'base_footprint'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        try:
            tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
            base_frame = 'base_link'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
            rospy.signal_shutdown("tf Exception")

    # Odometry
    (position, rotation) = get_odom()
    init_x, init_y = position.x, position.y
    last_rotation = 0
    linear_speed = 1
    angular_speed = 1
    
    
    # Move 40 centimeters
    moveDiscrete()
    
    time.sleep(0.65)
    
    moveDiscrete()
    
    time.sleep(0.65)

    (final_position, rotation) = get_odom()
    distance = sqrt(pow(init_x - final_position.x, 2) + pow(init_y - final_position.y, 2))
    print("Total Distance travelled ==> ", distance)

    break

































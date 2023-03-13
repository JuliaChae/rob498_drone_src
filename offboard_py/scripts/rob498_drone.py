#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped, TwistStamped
import mavros
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet

class Drone:
    def __init__(self):
        rospy.init_node('rob498_drone') 

        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)

        mavros.set_namespace()
        self.arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
        self.set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)

        self.srv_launch = rospy.Service('comm/launch', Empty, self.callback_launch)
        self.srv_test = rospy.Service('comm/test', Empty, self.callback_test)
        self.srv_land = rospy.Service('comm/land', Empty, self.callback_land)
        self.srv_abort = rospy.Service('comm/abort', Empty, self.callback_abort)
        self.service_mode = "INIT"

        self.pose = PoseStamped()
        self.vel = TwistStamped()

        self.state = State()
        prev_state = self.state

        if vicon_running():
            # Subscribe to the vicon topic /vicon/ROB498_Drone/ROB498_Drone
            rospy.Subscriber('vicon/ROB498_Drone/ROB498_Drone', PoseStamped, callback_vicon)
        else:
            # Use data from the RealSense camera
            pass

        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(20)
        # Wait for Flight Controller connection
        print(self.state)
        while (not rospy.is_shutdown() and not self.state.connected):
            self.rate.sleep()
        rospy.loginfo("Flight controller connected!")

        # Set max vertical velocity and max horizontal velocity
        max_velocity = 2.0
        rospy.wait_for_service('/mavros/param/set')
        try:
            max_hor_vel = rospy.ServiceProxy('/mavros/param/set', ParamSet)
            max_hor_vel(param_id="MPC_XY_VEL_ALL", value=ParamValue(real=max_velocity))
            print("Service max_horizontal_velocity (MPC_XY_VEL_ALL) call succeeded, velocity set to {}".format(max_velocity))
        except rospy.ServiceException as e:
            print("Service max_horizontal_velocity (MPC_XY_VEL_ALL) call failed: %s" % e)
        try:
            max_ver_vel = rospy.ServiceProxy('/mavros/param/set', ParamSet)
            max_ver_vel(param_id="MPC_Z_VEL_MAX_ALL", value=ParamValue(real=max_velocity))
            print("Service max_vertical_velocity (MPC_Z_VEL_ALL) call succeeded, velocity set to {}".format(max_velocity))
        except rospy.ServiceException as e:
            print("Service max_vertical_velocity (MPC_Z_VEL_ALL) call failed: %s" % e)

    def state_cb(self, msg):
        self.state = msg
    
    # Callback handlers
    def handle_launch(self):
        self.service_mode = "LAUNCH"
        print('Launch Requested. Your drone should take off.')
        
        self.pose.header.stamp = rospy.Time.now()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 1.5

        self.vel.twist.linear.x = 0
        self.vel.twist.linear.y = 0
        self.vel.twist.linear.z = 0.5

        return

    def handle_test(self):
        self.service_mode = "TEST"
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

    def handle_land(self):
        # If there is still stuff to execute in launch, wait for it to finish, then land 
        self.service_mode = "LAND"

        self.pose.header.stamp = rospy.Time.now()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0

        print('Land Requested. Your drone should land.')
        return 

    def handle_abort(self):
        # Land even if stuff left to do in launch 
        self.service_mode = "ABORT"

        self.pose.header.stamp = rospy.Time.now()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        
        return 

    # Service callbacks
    def callback_launch(self, request):
        self.handle_launch()
        return EmptyResponse()

    def callback_test(self, request):
        self.handle_test()
        return EmptyResponse()

    def callback_land(self, request):
        self.handle_land()
        return EmptyResponse()

    def callback_abort(self, request):
        self.handle_abort()
        return EmptyResponse()

def callback_vicon(vicon_msg):
    pass

def vicon_running(topic_name='vicon/ROB498_Drone/ROB498_Drone'):
    # Get a list of tuples containing the names and data types of all the topics that are currently published
    published_topics = rospy.get_published_topics()

    # Check if the topic exists by searching for its name in the list of published topics
    if any(topic_name in topic for topic in published_topics):
        #print(f"The topic '{topic_name}' exists, using vicon.")
        return True
    else:
        #print(f"The topic '{topic_name}' does not exist, not using vicon.")
        return False

if __name__ == "__main__":

    drone = Drone()
    last_request = rospy.Time.now()
    while not rospy.is_shutdown():
        if drone.service_mode == "LAUNCH":
            drone.pose.header.stamp = rospy.Time.now()
            drone.pose.pose.position.x = 0
            drone.pose.pose.position.y = 0
            drone.pose.pose.position.z = 1.5
        
        if drone.service_mode == "LAND" or drone.service_mode == "ABORT":
            drone.pose.header.stamp = rospy.Time.now()
            drone.pose.pose.position.x = 0
            drone.pose.pose.position.y = 0
            drone.pose.pose.position.z = 0

        if drone.state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.)):
            drone.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = rospy.Time.now() 
        else:
            if not drone.state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.)):
                drone.arming_client(True)
                last_request = rospy.Time.now()

        # Publish the position setpoint
        drone.local_pos_pub.publish(drone.pose)
        # Publish the velocity setpoint
        drone.local_vel_pub.publish(drone.vel)

        drone.rate.sleep()

    rospy.spin()

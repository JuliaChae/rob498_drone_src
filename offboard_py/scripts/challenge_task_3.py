import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped
import mavros
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# Once the TA confirms that your drone is stable, the ground control server will send a TEST command,
# along with a test identifier, the series of seven waypoints, and the current position of the drone in the
# Vicon reference frame.

class Drone:
    def __init__(self):
        rospy.init_node('rob498_drone') 

        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
        set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)

        self.srv_launch = rospy.Service('comm/launch', Empty, self.callback_launch)
        self.srv_test = rospy.Service('comm/test', Empty, self.callback_test)
        self.srv_land = rospy.Service('comm/land', Empty, self.callback_land)
        self.srv_abort = rospy.Service('comm/abort', Empty, self.callback_abort)

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
        while (not rospy.is_shutdown() and not self.state.connected):
            self.rate.sleep()
        rospy.loginfo("Flight controller connected!")

        # Set the mode to OFFBOARD
        now = rospy.get_rostime()
        if self.state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        else:
            if not self.state.armed and (now - last_request > rospy.Duration(5.)):
               arming_client(True)
               last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != self.state.armed:
            rospy.loginfo("Vehicle armed: %r" % self.state.armed)
        if prev_state.mode != self.state.mode: 
            rospy.loginfo("Current mode: %s" % self.state.mode)
        prev_state = self.state

        rospy.spin()

    def state_cb(self, msg):
        self.state = msg
    
    # Callback handlers
    def handle_launch(self):
        print('Launch Requested. Your drone should take off.')
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 1
        while not rospy.is_shutdown():
            # Publish the setpoint
            self.local_pos_pub.publish(pose)
            self.rate.sleep()

    def handle_test(self):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

    def handle_land(self):
        print('Land Requested. Your drone should land.')

    def handle_abort(self):
        print('Abort Requested. Your drone should land immediately due to safety considerations')

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
        print(f"The topic '{topic_name}' exists, using vicon.")
        return True
    else:
        print(f"The topic '{topic_name}' does not exist, not using vicon.")
        return False

if __name__ == "__main__":
    drone = Drone()
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from tf.transformations import quaternion_matrix

class ChallengeTask3:

    def __init__(self):
        self.STATE = 'INIT'
        self.WAYPOINTS = None
        self.WAYPOINTS_RECEIVED = False
        self.name = 'rob498_drone_05'  # Change 00 to your team ID
        self.pose = PoseStamped() # pose to set local_position to 
        self.current_state = State() 
        self.waypoint_cnt = -1 # Current idx of the waypoint
        self.current_waypoint = np.zeros((0,3)) # Waypoint position we are going 
        self.num_waypoints = 7 # Expected number of waypoints
        self.T_odom_vicon = np.eye(4) # Transform between odom and vicon frames
        self.current_pose = np.zeros((0,3)) # Current pose of the drone from /mavros/odometry/out

    def state_cb(self, msg):
        self.current_state = msg

    # Callback handlers
    def handle_launch(self):
        self.STATE = 'LAUNCH'
        print('Launch Requested.')

    def handle_test(self):
        self.STATE = 'TEST'
        print('Test Requested.')

    def handle_land(self):
        self.STATE = 'LAND'
        print('Land Requested.')

    def handle_abort(self):
        self.STATE = 'ABORT'
        print('Abort Requested.')

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

    def callback_waypoints(self, msg):
        if self.WAYPOINTS_RECEIVED:
            return
        print('Waypoints Received')
        self.WAYPOINTS_RECEIVED = True
        self.WAYPOINTS = np.empty((0,3))
        for pose in msg.poses:
            pos_h = np.array([pose.position.x, pose.position.y, pose.position.z, 1])
            pos_transformed = self.T_odom_vicon@pos_h.T # Transform from vicon frame to (pixhawk)odom frame
            pos = pos_transformed[:-1].T
            print("Waypoint added: \n")
            print("In Vicon frame: ", pos_h[:-1])
            print("In odom frame: ", pos)
            self.WAYPOINTS = np.vstack((self.WAYPOINTS, pos))

    def callback_vicon(self, vicon_msg):
        print("Received vicon message")
        print(vicon_msg)
        if self.STATE == 'INIT':
            # Construct rotation matrix between vicon and odom frames
            R = quaternion_matrix(vicon_msg.transform.quaterion)
            t = vicon_msg.transform.translation
            self.T_odom_vicon[:3,:3] = R
            self.T_odom_vicon[:3, 3] = t
    
    def callback_odom(self, odom_msg):
        self.current_pose = np.array([odom_msg.pose.position.x, odom_msg.pose.position.y, odom_msg.pose.position.z])


    def vicon_running(topic_name='vicon/ROB498_Drone/ROB498_Drone'):
        # Get a list of tuples containing the names and data types of all the topics that are currently published
        published_topics = rospy.get_published_topics()

        # Check if the topic exists by searching for its name in the list of published topics
        if any(topic_name in topic for topic in published_topics):
            print("using vicon.")
            return True
        else:
            print("not using vicon.")
            return False

    def start_up(self):
        # Do not change the node name and service topics!
        rospy.init_node(self.name)

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        self.odom_sub = rospy.Subscriber("mavros/odometry/out", Odometry, callback=self.callback_odom)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
        if self.vicon_running():
            # Subscribe to the vicon topic /vicon/ROB498_Drone/ROB498_Drone
            rospy.Subscriber('vicon/ROB498_Drone/ROB498_Drone', TransformStamped, self.callback_vicon)
        else:
            # Use data from the RealSense camera
            pass

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode) 
        self.srv_launch = rospy.Service(self.name+'/comm/launch', Empty, self.callback_launch)
        self.srv_test = rospy.Service(self.name+'/comm/test', Empty, self.callback_test)
        self.srv_land = rospy.Service(self.name+'/comm/land', Empty, self.callback_land)
        self.srv_abort = rospy.Service(self.name+'/comm/abort', Empty, self.callback_abort)

        self.sub_waypoints = rospy.Subscriber(self.name+'/comm/waypoints', PoseArray, self.callback_waypoints)

    # Main node
    def comm_node(self):

        self.start_up()

        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(20)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.current_state.connected):
            rate.sleep()

        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0

        # Send a few setpoints before starting
        for i in range(100):   
            if(rospy.is_shutdown()):
                break

            self.local_pos_pub.publish(self.pose)
            rate.sleep()

        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

        self.last_req = rospy.Time.now()

        print('This is the ROB498 Challenge 3 drone waypoint following node')
        while not rospy.is_shutdown():
            if self.WAYPOINTS_RECEIVED:
                print('Waypoints:\n', self.WAYPOINTS)
                self.waypoint_cnt = 0
                self.current_waypoint = self.WAYPOINTS[self.waypoint_cnt,:]

            # Your code goes here
            if self.STATE == 'INIT':
                print('Comm node: Initializing...')
                self.pose.pose.position.z = 0
            if self.STATE == 'LAUNCH':
                print('Comm node: Launching...')
                self.pose.pose.position.z = 1.15
            elif self.STATE == 'TEST' or (self.STATE == 'LAND' and self.waypoint_cnt < self.num_waypoints-1):
                print('Comm node: Testing...')

                # Set pose to current goal waypoint
                self.pose.pose.position.x = self.current_waypoint[0]
                self.pose.pose.position.y = self.current_waypoint[1]
                self.pose.pose.position.z = self.current_waypoint[2]

                # Check if we are within the 15 cm sphere of waypoint 
                if np.linalg.norm(self.current_waypoint - self.current_pose) < 0.15:

                    start_time = rospy.Time.now()

                    # If waypoint reached, hover for 5 seconds 
                    while(rospy.Time.now() - start_time) < rospy.Duration(5.0):
                        if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
                            if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                                rospy.loginfo("OFFBOARD enabled")
                            
                            self.last_req = rospy.Time.now()
                        else:
                            if(not self.current_state.armed and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
                                if(self.arming_client.call(self.arm_cmd).success == True):
                                    rospy.loginfo("Vehicle armed")
                            
                                self.last_req = rospy.Time.now()

                        self.local_pos_pub.publish(self.pose)

                    # Increment waypoint counter and waypoint                         
                    self.waypoint_cnt += 1
                    self.current_waypoint = self.WAYPOINTS[self.waypoint_cnt, :]

            elif self.STATE == 'LAND' or self.WAYPOINTS.shape[0] > self.num_waypoints-1:
                print('Comm node: Landing...')
                self.pose.pose.position.z = 0
            elif self.STATE == 'ABORT':
                print('Comm node: Aborting...')
                self.pose.pose.position.z = 0

            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
                if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")
                
                self.last_req = rospy.Time.now()
            else:
                if(not self.current_state.armed and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
                    if(self.arming_client.call(self.arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")
                
                    self.last_req = rospy.Time.now()

            self.local_pos_pub.publish(self.pose)

        rate.sleep()

if __name__ == "__main__":
    challenge = ChallengeTask3()
    challenge.comm_node()
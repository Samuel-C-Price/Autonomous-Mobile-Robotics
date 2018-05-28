################################################################
# Samuel Price - PRI15595313 - 15595313@students.lincoln.ac.uk # 
################################################################
import rospy, cv2, cv2.cv, cv_bridge, numpy
#from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan, Image 
from geometry_msgs.msg import PoseStamped, Twist

class Assignment:
    def laserCall(self, data):
        #distance set to be later used to stop the turtlebot 1 metre away from object.         
        self.distance = min(data.ranges)

    def __init__(self):
        self.bridge = cv_bridge.CvBridge() #converting between ROS images and opencv images
        #image windows for locating object
        cv2.namedWindow("image", 1)
        # Publishers & Subscribers delcared to send or recieve data from the robot. various topics used to perform different tasks.
        self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=1) 
        self.lasers = rospy.Subscriber('/turtlebot/scan', LaserScan, self.laserCall)
        self.goal_pub = rospy.Publisher('/turtlebot/move_base_simple/goal', PoseStamped, queue_size=1)                                          
                                
        self.twist = Twist()
        #waypoints for navigation, points set using pose location (x,y,z), orientation varied to change turtlebots camera angle.   
        self.first_waypoint = [(1.5, -4.2, 0.0),(0.0, 0.0, 0.0, 1)],
        self.second_waypoint = [(0.0, 0.0, 0.0), (0.0, 0.0, -1, 1.6)],
        self.third_waypoint = [(-1.3, 4.2, 0.0),(0.0, 0.0, 0.0, 1)], 
        self.fourth_waypoint = [(-4.3, -0.9, 0.0),(0, 0, 0.0, 1)],                        
                               
        #points on the map, robot will navigate to each point and then search for object (colour), once arrived at waypoint, flag will be set to True.
        self.point1 = False
        self.point2 = False
        self.point3 = False
        self.point4 = False
        # colours set to false, once found colours are set to true
        self.found_red = False
        self.found_blue = False
        self.found_green = False
        self.found_yellow = False        
        # navAroundMap is set to True as robot will begin by moving to first waypoint.
        # objSearch is set to False, once robot as reached the first waypoint, switch mode to search for first object. 
        self.nav = True
        self.objSearch = False

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)# Colour conversion from BGR to HSV #
        # Colour ranges, lower and upper bounds to discover objects. 
        yellowMask = cv2.inRange(hsv, numpy.array((30, 100, 100)), numpy.array((50, 255, 255)))
        redMask = cv2.inRange(hsv, numpy.array((0, 100, 100)), numpy.array((5, 255, 140)))
        blueMask = cv2.inRange(hsv, numpy.array((110, 50, 50)), numpy.array((130, 255, 255)))
        greenMask = cv2.inRange(hsv, numpy.array((60, 100, 50)), numpy.array((62, 255, 255)))
        # Image dimensions for calculations
        h, self.w, d = image.shape
        
        # Find objects in image, calculates the area of the object and the center, white circle will be displayed in the center. 
        yellow_M = cv2.moments(yellowMask)
        red_M = cv2.moments(redMask)
        blue_M = cv2.moments(blueMask)
        green_M = cv2.moments(greenMask)        
        ## Navigation towards various waypoints which have been set on the map. 
        ## Once arrived the flag will be set to True. 
        ## Following this, the turtlebot will begin searching for a object close the the waypoint.
        ## when navigating robot will print to the screen. 
        if self.point1 == False and self.nav == True:
            print "Im on my way to point-1"
            for t in range(40): # time allowed to reach waypoint 
                for pose in self.first_waypoint:
                    goal = self.goal_pose(pose)
                    self.goal_pub.publish(goal)
                    rospy.sleep(1)
            self.point1 = True
            self.objSearch = True
            self.nav = False
            print "I AM NOW AT POINT 1"

        if self.point2 == False and self.nav == True:
            print "Im on my way to point-2"
            for t in range(50):# time allowed to reach waypoint 
                for pose in self.second_waypoint:
                    goal = self.goal_pose(pose)
                    self.goal_pub.publish(goal)
                    rospy.sleep(1)    
            self.point2 = True
            self.nav = False
            self.objSearch = True
            print "I AM NOW AT POINT 2"  

        if self.point3 == False and self.nav == True:
            print "Im on my way to point-3"
            for t in range(40):# time allowed to reach waypoint 
                for pose in self.third_waypoint:
                    goal = self.goal_pose(pose)
                    self.goal_pub.publish(goal)
                    rospy.sleep(1)    
            self.point3 = True
            self.nav = False
            self.objSearch = True
            print "I AM NOW AT POINT 3"

        if self.point4 == False and self.nav == True:
            print "Im on my way to point-4"
            for t in range(90):# time allowed to reach waypoint 
                for pose in self.fourth_waypoint:
                    goal = self.goal_pose(pose)
                    self.goal_pub.publish(goal)
                    rospy.sleep(1) 
            self.point4 = True
            self.nav = False
            self.objSearch = True
            print "I AM NOW AT POINT 4"
        ## When turtlebot reaches a waypoint, object sreach will begin. 
        ## If turtlebot cannot see object, it will rotate (twist) until object is found. 
        ## If object is within the camera angle, turtlebot will move towards object.
        ## Once 1 metre away, it will stop and print to the screen the colour of the object found.
        ## To ensure the colour is not found again, flags are used to set to True once found. 
        if self.objSearch == True:       
            if red_M and yellow_M and blue_M and green_M['m00'] == 0:
                    self.twist.angular.z = 0.4 # rotate if object cannot be seen. 
                    self.cmd_vel_pub.publish(self.twist) # Publish this movement 
            
        if self.objSearch == True and self.found_red == False:
            if red_M['m00'] > 0: #If object has an area greater than 0 within the red colour range.
                cx = int(red_M['m10']/red_M['m00'])
                cy = int(red_M['m01']/red_M['m00'])
                cv2.circle(image, (cx, cy), 20, (255, 255, 255), -1) # white circle displayed when turtlebot is moving towards object.
                # Navigation
                if(self.distance >= 1):
                    err = cx - self.w/2
                    if (cy >= 100): #crude way to stop
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = -float(err) / 100
                        self.cmd_vel_pub.publish(self.twist)
                if (self.distance <= 1):# when distance equals 1 or less stop.
                    print "I am at the  Red pillar"
                    self.nav = True
                    self.objSearch = False
                    self.found_red = True
                # Display image
                cv2.imshow("image", image)
                cv2.waitKey(3)
        if self.objSearch == True and self.found_yellow == False:
            if yellow_M['m00'] > 0: #If object has an area greater than 0 and within the yellow colour range. 
                cx = int(yellow_M['m10']/yellow_M['m00'])
                cy = int(yellow_M['m01']/yellow_M['m00'])
                cv2.circle(image, (cx, cy), 20, (255, 255, 255), -1) # white circle displayed when turtlebot is moving towards object.
                # Navigation
                if(self.distance >= 1):
                    err = cx - self.w/2
                    if (cy >= 100):
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = -float(err) / 100
                        self.cmd_vel_pub.publish(self.twist)
                if (self.distance <= 1): # when distance equals 1 or less stop.
                    print "I am at the  Yellow pillar"
                    self.nav = True
                    self.objSearch = False
                    self.found_yellow = True
                # Display what robot is moving to
                cv2.imshow("image", image)
                cv2.waitKey(3)
        if self.objSearch == True and self.found_blue == False:
            if blue_M['m00'] > 0: #If object has an area greater than 0 and within the blue colour range. 
                cx = int(blue_M['m10']/blue_M['m00'])
                cy = int(blue_M['m01']/blue_M['m00'])
                cv2.circle(image, (cx, cy), 20, (255, 255, 255), -1) # white circle displayed when turtlebot is moving towards object.
                # Navigation
                if(self.distance >= 1):
                    err = cx - self.w/2
                    if (cy >= 100):
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = -float(err) / 100
                        self.cmd_vel_pub.publish(self.twist)
                if (self.distance <= 1): # when distance equals 1 or less stop.
                    print "I am at the blue pillar"
                    self.nav = True
                    self.objSearch = False
                    self.found_blue = True
                # Display image
                cv2.imshow("image", image)
                cv2.waitKey(3)
        if self.objSearch == True and self.found_green == False:       
            if green_M['m00'] > 0: #If object has an area greater than 0 and within the green colour range. 
                cx = int(green_M['m10']/green_M['m00'])
                cy = int(green_M['m01']/green_M['m00'])
                cv2.circle(image, (cx, cy), 20, (255, 255, 255), -1) # white circle displayed when turtlebot is moving towards object.
                # Navigation
                if(self.distance >= 1):
                    err = cx - self.w/2
                    if (cy >= 100):
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = -float(err) / 100
                        self.cmd_vel_pub.publish(self.twist)
                if (self.distance <= 1): # when distance equals 1 or less stop.
                    print "I am at the  Green pillar"
                    self.nav = True
                    self.objSearch = False
                    self.found_green = True
                # Display image
                cv2.imshow("image", image)
                cv2.waitKey(3)
    ## Goal positions set using waypoints set above. 
    ## Published depending on which waypoints have been visited.     
    def goal_pose(self, pose):
        next_goal = PoseStamped()
        next_goal.header.frame_id = '/map'
        next_goal.pose.position.x = pose[0][0]
        next_goal.pose.position.y = pose[0][1]
        next_goal.pose.position.z = pose[0][2]
        next_goal.pose.orientation.x = pose[1][0]
        next_goal.pose.orientation.y = pose[1][1]
        next_goal.pose.orientation.z = pose[1][2]
        next_goal.pose.orientation.w = pose[1][3]
        return next_goal 
        
rospy.init_node('assignment')
assignment = Assignment()
rospy.spin()          
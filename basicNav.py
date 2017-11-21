#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from math import *

class BasicNav:
	def __init__(self, scan_topic, pub_topic, heading_topic):
		self.scan_topic_name = scan_topic
		self.pub_topic_name = pub_topic
		self.heading_topic_name = heading_topic
		self.distance = 0.0

		#test code goal (children's hospital)
		self.lat1, self.lon1 = map(radians, [42.369045,-71.250569])

		#real code, to be implemnted with goals
		#self.lon1, self.lat1 = map(radians, [self.goalLon, self.goalLat])

	def gpsCallback(self,msg):
		self.lat2, self.lon2 = map(radians, [msg.latitude, msg.longitude])
		self.distance = self.calculateDistance(self.lat1,self.lon1,self.lat2,self.lon2)
		self.bearing = self.calculateBearing(self.lat1,self.lon1,self.lat2,self.lon2)

		print "--------------------"
		print "Horizontal Distance:"
		print self.distance
		print "--------------------"
		print "Bearing:"
		print self.bearing
		print "--------------------"

	def headingCallback(self,msg):
		#get robot heading data
		self.heading = (msg.data - 15) % 360

		#set up a twist to publish
		t = Twist()
		t.linear.x = 0.1
		t.linear.y = 0
		t.linear.z = 0
		t.angular.x = 0
		t.angular.y = 0
		t.angular.z = 0

		#if not decently close to the goal gps coordinate
		if self.distance > 10:
			#check difference of bearing and heading and take the modulo
			#modulo normalizes this difference
			#when bearing == heading, this will return 0
			norm_dif = (self.bearing - self.heading) % 360

			#if norm_dif is greater than 180, then robot heading to the right of the bearing
			#when this is true, the robot needs to turn left (speed based on angle)
			if norm_dif > 180:
				t.angular.z = max(((-norm_dif % 360) / 180), .1)
				print "turn left"
				print self.bearing
				print self.heading
				print "------------"
			#if norm_dif is less than 180, then robot heading to the left of the bearing
			#when this is true, the robot needs to turn right (speed based on angle)
			elif norm_dif < 180:
				t.angular.z = min(-(norm_dif) / 180, -.1) 
				print "turn right"
				print self.bearing
				print self.heading
				print "------------"
			#publish twist with updated values
			self.pub.publish(t)
		else:
			#publish 0 velocity twist message to stop the robot, goal reached
			self.pub.publish(t)	

	def start(self):
		#create publisher for twist messages
		self.pub = rospy.Publisher(self.pub_topic_name, Twist, queue_size=10)

		#subscribe to gps data and heading data
		rospy.Subscriber(self.scan_topic_name,NavSatFix,self.gpsCallback)
		rospy.Subscriber(self.heading_topic_name,Float64,self.headingCallback)
		rospy.spin()

	#calculates distance in meters bewtween 2 gps coordinates in radians
	def calculateDistance(self,lat1,lon1,lat2,lon2):
		self.dlon = lon2 - lon1
		self.dlat = lat2 - lat1
		self.a = sin(self.dlat/2)**2 + cos(lat1) * cos(lat2) * sin(self.dlon/2)**2
		self.c = 2 * asin(sqrt(self.a))
		return 6371000 * self.c #6371000 is the earth's radius in meters

	#calculates the bearing(direction to head towards) between 2 gps coordinates in radians
	def calculateBearing(self,lat1,lon1,lat2,lon2):
		bearing =atan2(cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1), sin(lon2-lon1)*cos(lat2))
		bearing =(degrees(bearing)-90)%360 #subtract 90 since this bearing has 0 degrees facing east
		return bearing

#init the node, set the topic names that it subscribes/publishes to, run the node's start method
def main():
	rospy.init_node('BasicNav')
	sensor_data = BasicNav("gps/fix","cmd_vel","heading")
	sensor_data.start()

#makes sure main method is run
if __name__ == '__main__':
	main()

#commands to publish messages in /gps/filtered topic
#these can be used to test the gpsCallback
#-------------
#1 pollack
#2 a random house relatively near moody st.
#3 intersection of south st. and main st.
#-------------
#1 rostopic pub /gps/filtered sensor_msgs/NavSatFix '{latitude: 42.365111, longitude: -71.262562}'
#2 rostopic pub /gps/filtered sensor_msgs/NavSatFix '{latitude: 42.368947, longitude: -71.232679}'
#3 rostopic pub /gps/filtered sensor_msgs/NavSatFix '{latitude: 42.375926, longitude: -71.250038}'


#commands to publish messages in /theHeading topic
#these can be used to test the headingCallback
#change the value for 'data' to change the direction your robot is facing (0.0 should be north)
#rostopic pub /theHeading std_msgs/Float64 '{data: 250.0}'

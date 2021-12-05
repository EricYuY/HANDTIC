#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from time import sleep
import paho.mqtt.client as mqtt_client
import time

host="192.168.1.55"
# host="10.228.6.125"
port = 1883

def connect_mqtt():
	def on_connect(client, userdata, flags, rc):
		if rc == 0:
			print("Connected to MQTT Broker!")
		else:
			print("Failed to connect, return code %d\n", rc)
	client = mqtt_client.Client()
	client.on_connect = on_connect
	client.connect(host, port)
	return client

def callback_publisher(data):

    global min_dist
    ranges = data.ranges
    min_dist = min(ranges)
    print(f"Min dist: {min_dist} m")
    client.publish(topic_lidar,str(min_dist))
          
def subscribe(client, topic, publisher):
	
    def on_message(client, userdata, msg):

        # Receive the angles
        print(f"Received: {msg.payload.decode()}: from {msg.topic} topic")
        pitch,roll = (msg.payload.decode()).split(',',2)
        pitch = float(pitch)
        roll = float(roll[1:])
        print(f"Pitch: {pitch}")
        print(f"Roll: {roll}")
        
        # Get the movement according to the angles
        threshold = 20
        if (abs(pitch)>threshold and abs(roll)<threshold):
            movement = 'translate'
            vel = get_velocity(pitch)
        elif (abs(roll)>threshold and abs(pitch)<threshold):
            movement = 'rotate'
            vel = get_velocity(roll)
        elif (abs(roll)<threshold and abs(pitch)<threshold):
            movement = 'quiet'
            vel = 0

        # Move the robot
        try:
            move_robot(movement,vel,publisher)
        except Exception as e:
            movement = 'quiet'
            vel = 0
            move_robot(movement,vel,publisher)


    client.subscribe(topic)
    client.on_message = on_message

def move_robot(movement, vel, publisher):

    # Create type of message to publish
    msg = Twist()

    # Move according to the 'movement' variable
    if movement == 'translate':
        print('Translating')
        msg.linear.x = vel
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
    elif movement == 'rotate':
        print('Rotating')
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = vel
    elif movement == 'quiet':
        print('Quiet')
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0    

    # Publish the message on topic
    publisher.publish(msg)
    # rospy.loginfo('Msg published.')

def get_velocity(data):
    if(data>-90 and data<-45):
        return 0.6
    elif(data>=-45 and data<0):
        return 0.3
    elif(data>=0 and data<45):
        return -0.3
    elif(data>=45 and data<90):
        return -0.6
    else:
        return 0
        

min_dist_lidar = None
# Topics from MQTT communication
topic_orientation="guante/orientacion"
topic_lidar="guante/dist_lidar"
# Client object for communication
client = connect_mqtt()

if __name__ == '__main__':

    # Initialize the ROS node for teleoperation
    rospy.init_node('teleop')

    # Suscribe to the command velocity topic
    publisher_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Suscribe and Publish to MQTT topics
    client.loop_start()
    subscribe(client,topic_orientation,publisher_vel)
    while True:
        rospy.Subscriber('/scan', LaserScan, callback_publisher, queue_size=1)
        rospy.spin()
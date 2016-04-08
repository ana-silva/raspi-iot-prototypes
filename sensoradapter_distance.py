#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt
import sys
import time
from datetime import datetime
import json
import RPi.GPIO as GPIO

class ultraDist(object):
	def __init__(self, ptrig, pecho):
		self.ptrig = ptrig
		self.pecho = pecho
		#GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(pecho, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.setup(ptrig, GPIO.OUT)
		GPIO.output(ptrig, 0)

		print("Ultrasonic distance meter: portTrig=" + str(ptrig) + " portEcho=" + str (pecho))
	
	def getValue(self):
		GPIO.output(self.ptrig, 0)
		time.sleep(0.1)
		GPIO.output(self.ptrig, 1)
		time.sleep(0.00001)
		GPIO.output(self.ptrig, 0)	
	
		while(0 == GPIO.input(self.pecho)):
			start = time.time()
		while(1 == GPIO.input(self.pecho)):
			None

		delay = (time.time() - start) * 1000 * 1000
		time.sleep(0.1)
	
		return (delay / 58.0)	
    
#sensor IDs
id_distance_sensor_0 = "A0" #todo: generate id?

# topics
topic_distance = "distance"

# data for mqtt broker
# from args #todo

mq_org = "vwbkdm"
mq_host = sys.argv[1]
mq_type = "raspberrypi"
mq_id = ""
mq_authtoken = ""
mq_clientId = "d:" + mq_org + ":" + mq_type + ":" + mq_id

# init GPIO ports for the distance sensors
distA = ultraDist(int(sys.argv[2]), int(sys.argv[3])) # ptrig, pecho


# create MQTT client and set user name and password 
client = mqtt.Client(client_id=mq_clientId, clean_session=True, userdata=None, protocol=mqtt.MQTTv311)
#client.username_pw_set(username="use-token-auth", password=mq_authtoken)


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
        print("ClientID: " + mq_clientId + "; Connected with result code " + str(rc))

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
        print("message: ")
        print(msg.topic + " " + str(msg.payload))
        for i in range(100,10000,100):
            buzzer.buzz(10,i)

# set mqtt client callbacks
client.on_connect = on_connect
client.on_message = on_message


# publishes message to MQTT broker
def sendMessage(topic, msg):
        client.publish(topic=topic, payload=msg, qos=0, retain=False)
	print(msg)

# connects to IBM IoT MQTT Broker
client.connect(mq_host, 1883, 60)



#runs a thread in the background to call loop() automatically.
#This frees up the main thread for other work that may be blocking.
#This call also handles reconnecting to the broker.
#Call loop_stop() to stop the background thread.
client.loop_start()

while True:
	print("---------------------------------------")
	# messages in json format
        # send message, topic: distance
	t = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
	msg_distance_sensor_0 = { "sensorID": id_distance_sensor_0,
                                  "timestamp": t, 
                                  "distance": "%.1f" % (distA.getValue())}
	sendMessage (topic_distance, json.dumps(msg_distance_sensor_0))  
        
	time.sleep(1)
        
client.loop_stop()
client.disconnect()


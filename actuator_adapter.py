#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt
import sys
import time
from datetime import datetime
import json
import serial
 
ser = serial.Serial('/dev/ttyUSB0',115200)

# topics
topic_action = 'action'

# data for mqtt broker
# from args #todo

mq_org = "vwbkdm"
mq_host = sys.argv[1]
mq_type = "raspberrypi"
mq_id = "listener"
mq_authtoken = ""
mq_clientId = "d:" + mq_org + ":" + mq_type + ":" + mq_id

# create MQTT client and set user name and password 
client = mqtt.Client(client_id=mq_clientId, clean_session=True, userdata=None, protocol=mqtt.MQTTv311)
#client.username_pw_set(username="use-token-auth", password=mq_authtoken)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
        print("ClientID: " + mq_clientId + "; Connected with result code " + str(rc))
	client.subscribe(topic_action)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
        print("message: ")
        print(msg.topic + " " + str(msg.payload))
        if (msg.topic=='action'):
		if (msg.payload == 'ON'):
			# send command through serial interface
			ser.write(b'ON')
		elif (msg.payload == 'OFF'):
			ser.write(b'OFF')

# set mqtt client callbacks
client.on_connect = on_connect
client.on_message = on_message


# publishes message to MQTT broker
def sendMessage(topic, msg):
        client.publish(topic=topic, payload=msg, qos=0, retain=False)
	print(msg)

# connects to IBM IoT MQTT Broker
client.connect(mq_host, 1883, 60)

client.loop_forever()

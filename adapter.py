#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt
import sys
import time
from datetime import datetime
import json
import RPi.GPIO as GPIO
import spidev

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

HexDigits = [0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x79,0x71]

ADDR_AUTO = 0x40
ADDR_FIXED = 0x44
STARTADDR = 0xC0
BRIGHT_DARKEST = 0
BRIGHT_TYPICAL = 2
BRIGHT_HIGHEST = 7
OUTPUT = GPIO.OUT
INPUT = GPIO.IN
LOW = GPIO.LOW
HIGH = GPIO.HIGH

"""
Ultrasonic distance sensor
"""
class ultraDist(object):
	def __init__(self, ptrig, pecho):
		self.ptrig = ptrig
		self.pecho = pecho
		GPIO.setup(pecho, INPUT, pull_up_down=GPIO.PUD_DOWN)
		GPIO.setup(ptrig, OUTPUT)
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

"""
Analog in (on linker-base ADC)
0: JP1 connector,
2: JP2 connector
"""
class analogInputReader(object):
        def __init__(self):
                self.spi = spidev.SpiDev()
                self.spi.open(0,0)
 
        def readadc (self, adPin):
                # read SPI data from MCP3004 chip, 4 possible adc’s (0 thru 3)
                if ((adPin > 3) or (adPin < 0)):
                        return -1
                r = self.spi.xfer2([1,8+adPin <<4,0])
                #print(r)
                adcout = ((r[1] &3) <<8)+r[2]
                return adcout
 
        def getLevel (self, adPin):
                value = self.readadc(adPin)
                volts = (value*3.3)/1024
                return (volts, value)
	
        def getTemperature (self):
                v0 = self.getLevel(0)
                temp = (((v0[0] * 1000) - 500)/10) # celsius
                return temp
        
        def getSoundStrength (self):
               v1 = self.getLevel(2)
               return v1[1]

"""
Linker Digital Display
"""


class TM1637:
	__doublePoint = False
	__Clkpin = 0
	__Datapin = 0
	__brightnes = BRIGHT_TYPICAL;
	__currentData = [0,0,0,0];
	
	def __init__( self, pinClock, pinData, brightnes ):
		self.__Clkpin = pinClock
		self.__Datapin = pinData
		self.__brightnes = brightnes;
		GPIO.setup(self.__Clkpin,OUTPUT)
		GPIO.setup(self.__Datapin,OUTPUT)
	# end  __init__

	def Clear(self):
		b = self.__brightnes;
		point = self.__doublePoint;
		self.__brightnes = 0;
		self.__doublePoint = False;
		data = [0x7F,0x7F,0x7F,0x7F];
		self.Show(data);
		self.__brightnes = b;				# restore saved brightnes
		self.__doublePoint = point;
	# end  Clear

	def Show( self, data ):
		for i in range(0,4):
			self.__currentData[i] = data[i];
		
		self.start();
		self.writeByte(ADDR_AUTO);
		self.stop();
		self.start();
		self.writeByte(STARTADDR);
		for i in range(0,4):
			self.writeByte(self.coding(data[i]));
		self.stop();
		self.start();
		self.writeByte(0x88 + self.__brightnes);
		self.stop();
	# end  Show

	def Show1(self, DigitNumber, data):	# show one Digit (number 0...3)
		if( DigitNumber < 0 or DigitNumber > 3):
			return;	# error
	
		self.__currentData[DigitNumber] = data;
		
		self.start();
		self.writeByte(ADDR_FIXED);
		self.stop();
		self.start();
		self.writeByte(STARTADDR | DigitNumber);
		self.writeByte(self.coding(data));
		self.stop();
		self.start();
		self.writeByte(0x88 + self.__brightnes);
		self.stop();
	# end  Show1
		
	def SetBrightnes(self, brightnes):		# brightnes 0...7
		if( brightnes > 7 ):
			brightnes = 7;
		elif( brightnes < 0 ):
			brightnes = 0;

		if( self.__brightnes != brightnes):
			self.__brightnes = brightnes;
			self.Show(self.__currentData);
		# end if
	# end  SetBrightnes

	def ShowDoublepoint(self, on):			# shows or hides the doublepoint
		if( self.__doublePoint != on):
			self.__doublePoint = on;
			self.Show(self.__currentData);
		# end if
	# end  ShowDoublepoint
			
	def writeByte( self, data ):
		for i in range(0,8):
			GPIO.output( self.__Clkpin, LOW)
			if(data & 0x01):
				GPIO.output( self.__Datapin, HIGH)
			else:
				GPIO.output( self.__Datapin, LOW)
			data = data >> 1
			GPIO.output( self.__Clkpin, HIGH)
		#endfor

		# wait for ACK
		GPIO.output( self.__Clkpin, LOW)
		GPIO.output( self.__Datapin, HIGH)
		GPIO.output( self.__Clkpin, HIGH)
		GPIO.setup(self.__Datapin, INPUT)
		
		while(GPIO.input(self.__Datapin)):
			time.sleep(0.001)
			if( GPIO.input(self.__Datapin)):
				GPIO.setup(self.__Datapin, OUTPUT)
				GPIO.output( self.__Datapin, LOW)
				GPIO.setup(self.__Datapin, INPUT)
			#endif
		# endwhile            
		GPIO.setup(self.__Datapin, OUTPUT)
	# end writeByte
    
	def start(self):
		GPIO.output( self.__Clkpin, HIGH) # send start signal to TM1637
		GPIO.output( self.__Datapin, HIGH)
		GPIO.output( self.__Datapin, LOW) 
		GPIO.output( self.__Clkpin, LOW) 
	# end start
	
	def stop(self):
		GPIO.output( self.__Clkpin, LOW) 
		GPIO.output( self.__Datapin, LOW) 
		GPIO.output( self.__Clkpin, HIGH)
		GPIO.output( self.__Datapin, HIGH)
	# end stop
	
	def coding(self, data):
		if( self.__doublePoint ):
			pointData = 0x80
		else:
			pointData = 0;
		
		if(data == 0x7F):
			data = 0
		else:
			data = HexDigits[data] + pointData;
		return data
	# end coding	
# end class TM1637

class buzzer
  pd = 27 #DATA PIN

  def __init__( self):
    GPIO.setup(pd, OUTPUT)
    GPIO.output(pd, 0)

  def buzzPwm(self, duration, freq):
    p = GPIO.PWM(pd,freq)
    p.start(50)
    time.sleep(float(duration)/1000)
    p.stop()

      # freq in Hz
  def buzz(self, duration, freq):
    GPIO.output(pd, 0)
    for i in range(1,duration):
      GPIO.output(pd, 1)
      time.sleep(0.001)
      GPIO.output(pd, 0)
      time.sleep(1.0/freq)

    GPIO.output(pd, 0)
# end class buzzer
 
#sensor IDs
id_distance_sensor_0 = "A0" #todo: generate id?
id_temperature_sensor_0 = "T0"

# topics
topic_distance = "distance"
topic_temperature = "temperature"
topic_action = 'action'

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

# init analog input reader
aiReader = analogInputReader()

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

# set mqtt client callbacks
client.on_connect = on_connect
client.on_message = on_message


# publishes message to MQTT broker
def sendMessage(topic, msg):
        client.publish(topic=topic, payload=msg, qos=0, retain=False)
	print(msg)

# connects to IBM IoT MQTT Broker
client.connect(mq_host, 1883, 60)

lastCmd  = '-'
display = TM1637(23,24,TM1637.BRIGHT_TYPICAL)
buzzer_act = buzzer()
for i in range (100, 10000, 100):
  buzzer_act.buzz(10,i)
  
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
	measured_distance = distA.getValue()
	#measured_distance = 1	
	msg_distance_sensor_0 = { "sensorID": id_distance_sensor_0,
                                  "timestamp": t, 
                                  "distance": "%.1f" % (measured_distance)}
	sendMessage (topic_distance, json.dumps(msg_distance_sensor_0))  
        
	# send message, topic: temperature
	t = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
	measured_temp = aiReader.getTemperature()
	msg_temperature_sensor_0 = { "sensorID": id_temperature_sensor_0,
                                     "timestamp": t, 
                                     "temperature": "%f" % (measured_temp)}
	sendMessage (topic_temperature, json.dumps(msg_temperature_sensor_0))

	if measured_temp > 25:
		if (lastCmd != 'ON'):
			sendMessage (topic_action, 'ON')
			lastCmd = 'ON'
	elif measured_temp < 24:
		if (lastCmd != 'OFF'):
			sendMessage (topic_action, 'OFF')
			lastCmd = 'OFF'

  display.ShowInt(int(measured_temp))
	time.sleep(1)
        
client.loop_stop()
client.disconnect()


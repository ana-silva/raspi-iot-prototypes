#!/bin/sh

#gets server IP
#SERVER_IP="$(ifconfig | grep -v 'eth0:' | grep -A 1 'eth0' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)"
#SERVER_IP="$(hostname -I)"

#passed through args
RASPI_IP=$1
SERVER_IP=$2
#gets pins
PIN_NR1 = $3 #Ptrig
PIN_NR2 = $4 #Pecho

#copy script (todo: using ssh key)
scp ./sensoradapter_distance.py pi@$RASPI_IP:~/Desktop/sensoradapter_distance.py;
#wget -r -A.py https://github.com/ana-silva/raspi-iot-prototypes/

#ssh connection
ssh pi@$RASPI_IP;

#install Mqtt client Paho
pip install paho-mqtt;

#start script
sudo python ~/Desktop/sensoradapter_distance.py $SERVER_IP $PIN_NR1 $PIN_NR2 &
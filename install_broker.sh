#!/bin/sh

# installing mosquitto in this machine

#sudo apt-get install python # install python if necessary

sudo apt-get install python-software-properties
#sudo apt-get install python3-software-properties
sudo apt-add-repository ppa:mosquitto-dev/mosquitto-ppa
sudo apt-get update
sudo apt-get install mosquitto mosquitto-clients python-mosquitto

#starting mosquitto
sudo mosquitto &

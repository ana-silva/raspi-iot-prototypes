# raspi-iot-prototypes
Contains example codes for sensor readings and sending comands to actuators attached to a Raspberry pi 

##installing mosquitto in ubuntu

$ sudo sh ./install_broker.sh


##installing distance sensor adapter in a Raspberry pi

$ sudo sh ./install_start_adapters.sh raspi_ip mqtt_broker_ip ptrig pecho

e.g., in Raspi2 -> ptrig=17, pecho=18 | ptrig=23, pecho=24

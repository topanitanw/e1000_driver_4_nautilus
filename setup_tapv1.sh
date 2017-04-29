#!/bin/bash -x

# delete the tap interface
sudo tunctl -d tap0
# create a tap0 interface 
sudo tunctl -t tap0
# establish a route 
sudo route add -net 10.10.10.0 netmask 255.255.255.0 eth0
sudo ifconfig tap0 up 10.10.10.2 netmask 255.255.255.0

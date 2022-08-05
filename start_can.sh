#!/bin/sh
sudo ip link set up can0 type can bitrate 500000
sudo ifconfig can0 txqueuelen 10000

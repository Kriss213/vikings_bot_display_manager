#!/bin/bash

echo "Delete remap the device serial port(ttyUSB*) to ttyDISPLAY"
echo "sudo rm   /etc/udev/rules.d/vikings_display.rules"
sudo rm   /etc/udev/rules.d/vikings_display.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
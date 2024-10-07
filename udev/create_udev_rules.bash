#!/bin/bash

parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )

cd "$parent_path"

for f in /usr/lib/udev/rules.d/*brltty*.rules; do
    sudo ln -sf /dev/null "/etc/udev/rules.d/$(basename "$f")"
done

echo "Remap the device serial port(ttyUSB*) to  ttyDISPLAY"
echo "Copying vikings_display.rules to  /etc/udev/rules.d/"
sudo cp ./vikings_display.rules  /etc/udev/rules.d
echo "Setting file permission to 644"
sudo chmod 644 /etc/udev/rules.d/vikings_display.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish "
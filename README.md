# Vikings Bot Display Manager

This ROS2 package updates Vikings Bot display with:
* robot name
* battery level
* robot internal status
* robot public status
* network parameters (signal strength, data usage)
* estimated charge time
* log

## Launch
```
ros2 launch vikings_bot_display_manager display_manager.launch.py
```

### Parameters
* ```vikings_bot_name``` - Namespace - name of the robot.
* ```net_interface``` - Network interface to show data usage. Use ```ip link show``` to find out.

## Topics

This node listens to these topics (listed without namespace):
* ```ping``` - Gets tobot internal and public status.
* ```battery_current``` - Determines whether robot is charging and how long will it charge.
* ```battery_charge``` - Gets battery precentage.

### Logging to display
* ```display_log``` - Publish to this topic to log text on display.
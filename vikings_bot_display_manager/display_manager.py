#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger
import serial
import time

from vikings_bot_interfaces.msg import Ping
from vikings_bot_firmware_py.mqtt_msgs import PingMqtt

import os
import threading
import psutil
import subprocess
import re

from std_msgs.msg import Int32, Int8, String

class NetworkMonitor:
    """
    Class automatically updates net_stats variable when initialized.
    """
    def __init__(self, interface:str, logger:RcutilsLogger, interval:int=1) -> None:
        self.interface = interface
        self.logger = logger

        self.__monitor_thread = threading.Thread(target=self.__get_network_bandwith)
        self.__monitor_thread.daemon = True
        self.__monitor_thread.start()

        self.__min_dbm = -100 # weak signal
        self.__max_dbm = -50 # strong signal

        self.net_stats:dict = {
            "up":0,
            "down":0,
            "signal_strength":0
        }
        self.interval = interval
        
    def __get_network_data(self):
        net_io = psutil.net_io_counters(pernic=True)
        
        if self.interface in net_io:
            stats = net_io[self.interface]
            return stats.bytes_sent, stats.bytes_recv
        else:
            self.logger.warning(f"Failed to find given network interface: {self.interface}", once=True)
            return None, None
    
    def __get_wifi_signal_strength(self):
        try:
            result = subprocess.run(['iwconfig'], capture_output=True, text=True)
            output = result.stdout
            match = re.search(r'Signal level=(-\d+) dBm', output)
            if match:
                signal_dbm = int(match.group(1))
                return signal_dbm
            else:
                return None
        except Exception as e:
            self.logger.warning(f"Failed to get network signal strength: {e}")
            return None

    def __get_network_bandwith(self):
        bytes_sent_prev, bytes_recv_prev = self.__get_network_data()
    
        while True:
            try:
                time.sleep(self.interval)
                bytes_sent_now, bytes_recv_now = self.__get_network_data()
                
                # Calculate bandwidth (in bytes) over the interval
                sent_diff = bytes_sent_now - bytes_sent_prev
                recv_diff = bytes_recv_now - bytes_recv_prev

                self.net_stats["up"] = (sent_diff / 1024) / self.interval
                self.net_stats["down"] = (recv_diff / 1024) / self.interval

                wifi_dbm = min(max(self.__get_wifi_signal_strength(),self.__min_dbm), self.__max_dbm)
                self.net_stats["signal_strength"] = int(100 * (wifi_dbm - self.__min_dbm) / (self.__max_dbm - self.__min_dbm))

                bytes_sent_prev, bytes_recv_prev = bytes_sent_now, bytes_recv_now
            except:
                continue

class DisplayManager(Node):
    """
    The purpose of this node is to update display with:
    * robot name
    * battery level
    * robot internal status
    * robot public status
    * network parameters (signal strenghth, data usage)
    * estimated charge time
    * log
    * TODO other
    """

    def __init__(self):
        super().__init__("DisplayManagerNode")

        self.declare_parameter('net_interface', value='')

        self.__net_interface = self.get_parameter('net_interface').value

        self.__PORT = "/dev/ttyDISPLAY"
        self.__BAUD = 9600

        self.__serial = None

        # thread to process incoming messages
        self.__rx_thread = threading.Thread(target=self.read_serial)
        self.__rx_thread.daemon = True

        while self.__serial == None:
            self.connect_serial()

        self.__network_monitor = NetworkMonitor(interface=self.__net_interface, logger=self.get_logger())
        
        # each sent instruction must be terminated with this value
        self.__terminator = b'\xFF\xFF\xFF'

        # information
        self.__robot_name:str = self.get_namespace()[1:]

        # Status
        self._internal_status:str = "Unkown"
        self._public_status:str = "Unknown"
        
        # battery
        self._battery_level:int = -1
        self._battery_current:int = 0
        self.is_charging:bool = False

        # network
        self.network_strength:int = 0
        self.network_up = 0
        self.network_down = 0
        
        # LOG
        self.__log_lines:list = []

        # subscribe to ros2 topics for information
        # Get STATUS from ping message
        self.ping_subscribtion = self.create_subscription(Ping,
            PingMqtt.mqtt_topic_name, self.__ping_callback, 3)

        # get battery current to determine if it's charging
        self.battery_current_subscription = self.create_subscription(
            Int32, "battery_current", self.__current_clb, 10)

        # get remaining battery %
        self.battery_charge_subscription = self.create_subscription(Int8,
            "battery_charge", self.__charge_clb, 10)

        # subscribe to display log info topic
        self.display_log_subscription = self.create_subscription(String,
            "display_log", self.__display_log_clb, 10)
        
        self.__instruction_buffer:list = []


        # A timer to update display
        self.__display_update_timer = self.create_timer(timer_period_sec=1, callback=self.__update_display)
 
    def __del__(self):
        # close serial communication
        self.__serial.close()

    def connect_serial(self):
        """
        Connect serial device.
        """
        try:
            self.__serial = serial.Serial(
                port=self.__PORT,
                baudrate=self.__BAUD
            )
            self.__rx_thread.start()
            self.get_logger().info("Serial initialized")
        except:
            self.get_logger().warn(f"Failed to connect serial. Retrying...", once=True)
            time.sleep(1)
        
    def read_serial(self):
        while True:
            if self.__serial.is_open:
                try:
                    data = self.__serial.readline()
                    if "SHTDWN" in str(data):
                        self.get_logger().info(f"SHUTDOWN REQUEST RECIEVED FROM DISPLAY. Sending shutdown signal to host...")
                        msg = String()
                        msg.data = "Shutting down computer..."
                        self.__display_log_clb(msg)
                        os.system('touch /tmp/shutdown_signal')
                    
                except serial.SerialException:
                    self.get_logger().error('Failed to read from serial port')
            else:
                # reconnect to serial
                self.connect_serial()

    @property
    def internal_status(self):
        return self._internal_status

    @internal_status.setter
    def internal_status(self, value: str):
        if value == self.internal_status:
            return
        self._internal_status = value
        update_cmd = f'int_status.txt=\"{self.internal_status}\"'
        self.__instruction_buffer.append(update_cmd)

    @property
    def public_status(self):
        return self._public_status

    @public_status.setter
    def public_status(self, value: str):
        if value == self.public_status:
            return
        self._public_status = value
        update_cmd = f'pub_status.txt=\"{self.public_status}\"'
        self.__instruction_buffer.append(update_cmd)

    @property
    def battery_level(self):
        return self._battery_level

    @battery_level.setter
    def battery_level(self, value: int):
        if value == self.battery_level:
            return
        self._battery_level = value
        update_cmds = [
            f'bat_level_bar.val={self.battery_level}',
            f'bat_level_pr.txt=\"{self.battery_level} %\"',
        ] 
        self.__instruction_buffer += update_cmds
    
    @property
    def battery_current(self):
        return self._battery_current

    @battery_current.setter
    def battery_current(self, value: int):
        if value == self.battery_current:
            return
        self._battery_current = value
        self.is_charging = value > 1000
        if self.is_charging:
            bat_text = 'Est. charge\r\ntime'
            update_cmds = [
                f"ch_rm_txt.txt=\"{bat_text}\"",
                f"ch_rm_time.txt=\"{self.__get_bat_charge_time(fstr=True)}\""]
        else:
            bat_text = ' '
            update_cmds = [
                f'ch_rm_txt.txt=\"{bat_text}\"',
                f'ch_rm_time.txt=\" \"',
            ]
        
        self.__instruction_buffer += update_cmds
    
    def __get_bat_charge_time(self, fstr:bool=False) -> tuple|str:
        """
        Calculate remaining charge time.

        :param fstr: If True, get formatted string, tuple otherwise.

        :return tuple: (h, min)'
        """
        if not  self.is_charging:
            # less than 1 A -> assume not charging
            return (0, 0)
        
        full_capacity = 145000 # mAh
        needed_capacity = full_capacity * (1 - self.battery_level/100)

        hours_float = needed_capacity / (self.battery_current + 1e-8)
        minutes = int((hours_float % 1) * 60)
        hours = int(hours_float)
        if fstr:
            if not (hours or minutes):
                return " "
        
            return f"{hours}h {minutes}min"   

        return (hours or minutes)

    def __send_command(self, command:str) -> bool:
        """
        Send command to display.

        :param command: Non-encoded command.
        
        :return bool: True if sent sucessfully, False otherwise
        """
        instruction = str.encode(command)
        try:
            if self.__serial.is_open:
                self.__serial.write(instruction + self.__terminator)
                return True
            else:
                self.connect_serial()
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")
        return False
    
    def __current_clb(self, msg:Int32) -> None:
        """
        Set battery current.
        :param msg: An Int32 ROS2 message.
        """
        self.battery_current = int(msg.data)
        
        self.is_charging = self.battery_current > 1000

    def __charge_clb(self, msg:Int8) -> None:
        """
        Set battery charge %.
        :param msg: An Int8 ROS2 message.
        """
        self.battery_level = int(msg.data)
    
    def __ping_callback(self, msg:Ping) -> None:
        """
        Callback function to update variables from Ping message.

        :param msg: A Ping message
        """
        self.internal_status = str(msg.internal_status).upper()
        self.public_status = str(msg.public_status).upper()

    def __display_log_clb(self, msg:String) -> None:
        """
        Set message to log to display.
        :param msg: A ROS2 String message.
        """
        cmd = f"log_box.txt+=\"{msg.data}\r\n\""
        
        if len(self.__log_lines) >= 12 or len("\r\n".join(self.__log_lines) ) >= 720:
            self.__log_lines.clear()
            self.__instruction_buffer.append(f"log_box.txt=\"\"")
        if len(msg.data) > 60: # breaks to new line
            self.__log_lines.append(msg.data[:60])
            self.__log_lines.append(msg.data[61:])
        else:
            self.__log_lines.append(msg.data)
        self.__instruction_buffer.append(cmd)

    def __update_display(self) -> None:
        # seperate instructions for robot name and connection status
        self.__instruction_buffer += [
            f"con_status.bco=2024",
            f"robot_name.txt=\"{self.__robot_name.replace('_',' ').upper()}\""
        ]

        # seperate instructions for network parameters
        self.network_up, self.network_down, self.network_strength = self.__network_monitor.net_stats.values()
        self.__instruction_buffer += [
            f"wifi_bar.val={self.network_strength}",
            f"wifi_pr.txt=\"{self.network_strength} %\"",
            f"up_speed.txt=\"{self.network_up:.0f} KB/s\"",
            f"down_speed.txt=\"{self.network_down:.0f} KB/s\"",
        ]

        for instruction in self.__instruction_buffer:
          while not self.__send_command(instruction):
              pass # loop until command is sent successfully
        self.__instruction_buffer = []


def main(args=None):
    rclpy.init(args=args)

    displayManagerNode = DisplayManager()
    
    try:
        rclpy.spin(displayManagerNode)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Clean shutdown: displayManagerNode")
    else:
        rclpy.shutdown()

    displayManagerNode.destroy_node()

if __name__ == "__main__":
    main()
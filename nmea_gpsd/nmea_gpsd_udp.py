#!/usr/bin/env python3

import socket
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
from sbg_driver.msg import SbgUtcTime

class NmeaToGpsd(Node):
    def __init__(self):
        super().__init__('nmea_to_gpsd')

        # Parameters
        self.declare_parameter('gpsd_host', '127.0.0.1')  # where gpsd listens for UDP
        self.declare_parameter('gpsd_port', 3001)         # UDP port gpsd listens on
        self.declare_parameter('nmea_topic', '/nmea')
        self.declare_parameter('utc_topic', '/sbg/utc_time')

        self.gpsd_host = self.get_parameter('gpsd_host').value
        self.gpsd_port = int(self.get_parameter('gpsd_port').value)
        self.nmea_topic = self.get_parameter('nmea_topic').value
        self.utc_topic = self.get_parameter('utc_topic').value

        # UDP socket (no connect/accept; connectionless)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Subscribe to NMEA sentences
        self.sub_nmea = self.create_subscription(
            Sentence,
            self.nmea_topic,
            self.nmea_callback,
            10
        )

        # Subscribe to SBG UTC time
        self.sub_utc = self.create_subscription(
            SbgUtcTime,
            self.utc_topic,
            self.utc_callback,
            10
        )

        self.get_logger().info(
            f'Forwarding NMEA from [{self.nmea_topic}] ' 
            f'and publishing GPRMC from [{self.utc_topic}] to gpsd at {self.gpsd_host}:{self.gpsd_port}'
        )

    def gprmc_from_utc(self, utc_msg, lat='0.0', lat_dir='N', lon='0.0', lon_dir='E', status='A'):
        """
        Generate GPRMC sentence using SbgUtcTime message.
        """
        t = datetime(utc_msg.year, utc_msg.month, utc_msg.day,
                     utc_msg.hour, utc_msg.min, utc_msg.sec,
                     int(utc_msg.nanosec / 1000), tzinfo=timezone.utc)
        utc_time_str = t.strftime('%H%M%S.%f')[:9]  # hhmmss.ss
        utc_date_str = t.strftime('%d%m%y')         # DDMMYY

        gprmc_core = f"GPRMC,{utc_time_str},,,,,,,,{utc_date_str},,,A"

        # Compute checksum
        checksum = 0
        for char in gprmc_core:
            checksum ^= ord(char)
        checksum_str = f"{checksum:02X}"

        gprmc_sentence = f"${gprmc_core}*{checksum_str}"
        return gprmc_sentence

    def nmea_callback(self, msg: Sentence):
        sentence = msg.sentence.strip()

        # Forward original GGA
        self.send_sentence(sentence)

    def utc_callback(self, msg: SbgUtcTime):
        # Publish GPRMC using latitude/longitude from last known GGA or defaults
        gprmc_sentence = self.gprmc_from_utc(msg)
        self.send_sentence(gprmc_sentence)

    def send_sentence(self, sentence):
        if not sentence.startswith('$'):
            return
        try:
            data = (sentence + '\r\n').encode('ascii')
            self.sock.sendto(data, (self.gpsd_host, self.gpsd_port))
        except OSError as e:
            self.get_logger().warn(f'Failed to send NMEA to gpsd: {e}')


def main():
    rclpy.init()
    node = NmeaToGpsd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
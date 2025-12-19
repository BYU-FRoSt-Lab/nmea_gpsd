#!/usr/bin/env python3

import socket
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence

class NmeaToGpsd(Node):
    def __init__(self):
        super().__init__('nmea_to_gpsd')

        # Parameters
        self.declare_parameter('gpsd_host', '127.0.0.1')  # where gpsd listens for UDP
        self.declare_parameter('gpsd_port', 3001)         # UDP port gpsd listens on
        self.declare_parameter('nmea_topic', '/nmea')

        self.gpsd_host = self.get_parameter('gpsd_host').value
        self.gpsd_port = int(self.get_parameter('gpsd_port').value)
        self.nmea_topic = self.get_parameter('nmea_topic').value

        # UDP socket (no connect/accept; connectionless)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.sub = self.create_subscription(
            Sentence,
            self.nmea_topic,
            self.nmea_callback,
            10
        )

        self.get_logger().info(
            f'Forwarding NMEA from [{self.nmea_topic}] ' 
            f'to gpsd via UDP at {self.gpsd_host}:{self.gpsd_port}'
        )

    def gprmc_from_gga(self, gga_sentence):
        """
        Generate minimal GPRMC sentence using GGA lat/lon and ROS2 UTC timestamp.
        This ensures the UTC time in GPRMC exactly matches the original GGA fix.
        """
        fields = gga_sentence.split(',')
        if len(fields) < 6:
            return None

        # Extract latitude/longitude from GGA
        lat = fields[2]
        lat_dir = fields[3]
        lon = fields[4]
        lon_dir = fields[5]

        # Fix quality
        fix_quality = int(fields[6]) if len(fields) > 6 and fields[6].isdigit() else 0
        status = 'A' if fix_quality > 0 else 'V'

        # Extract UTC time and date from ROS2 timestamp
        # t = datetime.fromtimestamp(timestamp.sec + timestamp.nanosec * 1e-9, tz=timezone.utc)
        # utc_time_str = t.strftime('%H%M%S.%f')[:9]  # hhmmss.ss
        # utc_date_str = t.strftime('%d%m%y')         # DDMMYY
        utc_time_str = fields[1]  # Use time from GGA

        # Construct GPRMC core
        gprmc_core = f"GPRMC,{utc_time_str},{status},{lat},{lat_dir},{lon},{lon_dir},0.0,0.0,{utc_date_str},,,A"

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

        # Construct GPRMC using ROS2 timestamp
        GPRMC_sentence = self.gprmc_from_gga(sentence)
        if GPRMC_sentence:
            self.send_sentence(GPRMC_sentence)

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
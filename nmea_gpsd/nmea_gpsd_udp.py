#!/usr/bin/env python3

import socket

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

    def nmea_callback(self, msg: Sentence):
        sentence = msg.sentence.strip()
        if not sentence.startswith('$'):
            return

        try:
            # NMEA lines with CRLF, as usual
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

#!/usr/bin/env python3

import socket
import time

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence



class NmeaToGpsd(Node):
    def __init__(self):
        super().__init__('nmea_to_gpsd')
        self.conn = None
        # Parameters
        self.declare_parameter('gpsd_host', '127.0.0.1')
        self.declare_parameter('gpsd_port', 3001)
        self.declare_parameter('nmea_topic', '/nmea')

        self.gpsd_host = self.get_parameter('gpsd_host').value
        self.gpsd_port = self.get_parameter('gpsd_port').value
        self.nmea_topic = self.get_parameter('nmea_topic').value

        self.get_logger().info(
            f'Connecting to gpsd at {self.gpsd_host}:{self.gpsd_port}'
        )

        self.sock = None
        self._connect()

        self.sub = self.create_subscription(
            Sentence,
            self.nmea_topic,
            self.nmea_callback,
            10
        )

        self.get_logger().info(
            f'Forwarding NMEA from [{self.nmea_topic}] '
            f'to gpsd at {self.gpsd_host}:{self.gpsd_port}'
        )

    def _setup_server(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.gpsd_host, self.gpsd_port))
        self.server.listen(1)

        self.get_logger().info(
            f'Listening for gpsd connection on port {self.gpsd_port}'
        )

        self.conn, addr = self.server.accept()
        self.get_logger().info(f'gpsd connected from {addr}')
    
    def _connect(self):
        while rclpy.ok():
            try:
                self.sock = socket.create_connection(
                    (self.gpsd_host, self.gpsd_port), timeout=5.0
                )
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.get_logger().info('Connected to gpsd socket')
                return
            except OSError as e:
                self.get_logger().warn(
                    f'Failed to connect to gpsd ({e}) on IP {self.gpsd_host} and port {self.gpsd_port}, retrying...'
                )
                time.sleep(2.0)

    def nmea_callback(self, msg: Sentence):
        if self.conn is None:
            return

        sentence = msg.sentence.strip()
        if not sentence.startswith('$'):
            return

        try:
            self.conn.sendall((sentence + '\r\n').encode('ascii'))
        except OSError:
            self.get_logger().warn('gpsd disconnected')
            self.conn.close()
            self.conn = None


def main():
    rclpy.init()
    node = NmeaToGpsd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

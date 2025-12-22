#!/usr/bin/env python3

import socket
import time

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence


class NmeaToGpsd(Node):
    def __init__(self):
        super().__init__('nmea_to_gpsd')

        # Parameters
        self.declare_parameter('gpsd_host', '0.0.0.0')  # bind address
        self.declare_parameter('gpsd_port', 3001)       # port gpsd will connect to
        self.declare_parameter('nmea_topic', '/nmea')

        self.gpsd_host = self.get_parameter('gpsd_host').value
        self.gpsd_port = int(self.get_parameter('gpsd_port').value)
        self.nmea_topic = self.get_parameter('nmea_topic').value

        self.server = None
        self.conn = None

        # Start TCP server in non-blocking style
        self._setup_server()

        # Timer to accept gpsd connection without blocking callbacks
        self.accept_timer = self.create_timer(0.5, self._accept_once)

        # ROS subscription
        self.sub = self.create_subscription(
            Sentence,
            self.nmea_topic,
            self.nmea_callback,
            10
        )

        self.get_logger().info(
            f'Forwarding NMEA from [{self.nmea_topic}] '
            f'to gpsd TCP client on {self.gpsd_host}:{self.gpsd_port}'
        )

    def _setup_server(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.gpsd_host, self.gpsd_port))
        self.server.listen(1)
        self.server.setblocking(False)
        self.get_logger().info(
            f'Listening for gpsd TCP connection on {self.gpsd_host}:{self.gpsd_port}'
        )

    def _accept_once(self):
        """Non-blocking accept; called periodically by a timer."""
        if self.conn is not None:
            return
        try:
            conn, addr = self.server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.conn = conn
            self.get_logger().info(f'gpsd connected from {addr}')
        except BlockingIOError:
            # No pending connection; ignore
            pass
        except OSError as e:
            self.get_logger().warn(f'Accept failed: {e}')

    def nmea_callback(self, msg: Sentence):
        if self.conn is None:
            return

        sentence = msg.sentence.strip()
        if not sentence.startswith('$'):
            return

        try:
            # gpsd expects standard NMEA line endings
            self.conn.sendall((sentence + '\r\n').encode('ascii'))
        except OSError as e:
            self.get_logger().warn(f'gpsd disconnected or send failed: {e}')
            try:
                self.conn.close()
            except OSError:
                pass
            self.conn = None

def main():
    rclpy.init()
    node = NmeaToGpsd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix

APOGEE = 6000.0  # [ft]

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('altimeter_data_pub')
        self.publisher_ = self.create_publisher(NavSatFix, 'altimeter/fused', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.time = -math.sqrt(APOGEE)

    def timer_callback(self):
        msg = NavSatFix()
        msg.altitude = (APOGEE - (self.time**2)) / 3.281
        self.time += self.timer_period
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.altitude * 3.281}ft')


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

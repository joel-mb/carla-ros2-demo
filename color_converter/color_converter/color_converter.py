# Copyright (c) 2023 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https:#opensource.org/licenses/MIT>.

import array

import rclpy
import rclpy.node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Image


CITYSCAPES_PALETTE_MAP = {
     0: [  0,   0,   0, 255],   # unlabeled
     1: [128,  64, 128, 255],   # road
     2: [244,  35, 232, 255],   # sidewalk
     3: [ 70,  70,  70, 255],   # building
     4: [102, 102, 156, 255],   # wall
     5: [190, 153, 153, 255],   # fence
     6: [153, 153, 153, 255],   # pole
     7: [250, 170,  30, 255],   # traffic light
     8: [220, 220,   0, 255],   # traffic sign
     9: [107, 142,  35, 255],   # vegetation
    10: [152, 251, 152, 255],   # terrain
    11: [ 70, 130, 180, 255],   # sky
    12: [220,  20,  60, 255],   # pedestrian
    13: [255,   0,   0, 255],   # rider
    14: [  0,   0, 142, 255],   # Car
    15: [  0,   0,  70, 255],   # truck
    16: [  0,  60, 100, 255],   # bus
    17: [  0,  80, 100, 255],   # train
    18: [  0,   0, 230, 255],   # motorcycle
    19: [119,  11,  32, 255],   # bicycle
    20: [110, 190, 160, 255],   # static
    21: [170, 120,  50, 255],   # dynamic
    22: [ 55,  90,  80, 255],   # other
    23: [ 45,  60, 150, 255],   # water
    24: [157, 234,  50, 255],   # road line
    25: [ 81,   0,  81, 255],   # ground
    26: [150, 100, 100, 255],   # bridge
    27: [230, 150, 140, 255],   # rail track
    28: [180, 165, 180, 255]    # guard raiL
}


class ColorConverter(rclpy.node.Node):
    def __init__(self):
        param = rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        super().__init__(
            "color_converter",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
            parameter_overrides=[param]
        )

        # _input = self.get_parameter('input').get_parameter_value().string_value
        # _output = self.get_parameter('ouput').get_parameter_value().string_value
        _input = "/carla/vehicles/hero/sensors/semantic_segmentation/front/image"
        _output = "/carla/vehicles/hero/sensors/semantic_segmentation/front/cityscapes/image"
        
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.VOLATILE
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self._subscriber = self.create_subscription(Image, _input, self._callback, qos)

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.VOLATILE
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self._publisher = self.create_publisher(Image, _output, 
                                                qos_profile=qos)

    def _callback(self, data):
        cityscapes = data
        for i in range(0, len(cityscapes.data), 4):
            cityscapes.data[i:i+4] = array.array('B', CITYSCAPES_PALETTE_MAP.get(cityscapes.data[i+2], CITYSCAPES_PALETTE_MAP[0]))
        self._publisher.publish(cityscapes)


def main(args=None):
    rclpy.init(args=args)
    node = ColorConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

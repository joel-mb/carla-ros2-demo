# Copyright (c) 2023 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import json
import threading

import rclpy
import rclpy.node
import rclpy.qos

import carla

from sensor_msgs.msg import Image, PointCloud2

THRESHOLD = 0.001
SKIP_TICKS = 100
DURATION_TICKS = 1200
FIXED_DELTA_SECONDS = 0.05

SENSORS_TIMEOUT = 1.0


class ROS2Stack(rclpy.node.Node):

    def __init__(self, world):
        param = rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        super().__init__(
            "ros_demo",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
            parameter_overrides=[param]
        )

        self._world = world
        self._map = world.get_map()
        self._blueprint_library = world.get_blueprint_library()

        self.vehicle, self.sensors = self._setup()

    def _setup(self):
        with open(self.get_parameter('objects_definition_file').get_parameter_value().string_value, 'r') as f:
           config = json.load(f)

        vehicle = self._setup_vehicle(config)
        sensors = self._setup_sensors(vehicle, config.get("sensors", []))

        return vehicle, sensors

    def _setup_vehicle(self, config):
        self.get_logger().info("Spawning vehicle...")

        bp = self._blueprint_library.filter(config.get("type"))[0]
        bp.set_attribute("role_name", config.get("id"))
        bp.set_attribute("ros_name", config.get("id")) 

        return  self._world.spawn_actor(
            bp,
            self._map.get_spawn_points()[0],
            attach_to=None)

    def _callback(self, data):
        stamp = data.header.stamp
        self.sensors_queue.put_nowait(stamp.sec + stamp.nanosec*1e-9)

    def _setup_sensors(self, vehicle, sensors_config):
        sensors = []

        for sensor in sensors_config:
            self.get_logger().info("Spawning sensor: {}".format(sensor))

            bp = self._blueprint_library.filter(sensor.get("type"))[0]
            bp.set_attribute("ros_name", sensor.get("id")) 
            bp.set_attribute("role_name", sensor.get("id")) 
            for key, value in sensor.get("attributes", {}).items():
                bp.set_attribute(str(key), str(value))

            wp = carla.Transform(
                location=carla.Location(
                    x=sensor["spawn_point"]["x"],
                    y=-sensor["spawn_point"]["y"],
                    z=sensor["spawn_point"]["z"]
                ),
                rotation=carla.Rotation(
                    roll=sensor["spawn_point"]["roll"],
                    pitch=-sensor["spawn_point"]["pitch"],
                    yaw=-sensor["spawn_point"]["yaw"]
                )
            )

            sensors.append(
                self._world.spawn_actor(
                    bp,
                    wp,
                    attach_to=vehicle
                )
            )

            sensors[-1].enable_for_ros()

            # topic_name = "/carla/{}/{}".format(VEHICLE_CONFIGURATION["id"], sensor["id"])
            # if sensor["ros"]["name"]:
            #     topic_name += "/" + sensor["ros"]["name"] 

            # self.create_subscription(
            #     sensor["ros"]["msg"],
            #     topic_name,
            #     self._callback,
            #     qos_profile=10
            # )

        return sensors

    def destroy_node(self):
        self.get_logger().info("Destroying carla actors...")

        for sensor in self.sensors:
            sensor.destroy()
        self.vehicle.destroy()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    client = carla.Client("localhost", 2000)
    client.set_timeout(60.0)

    world = client.get_world()
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    world.apply_settings(settings)

    node = ROS2Stack(world)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    spin_thread.start()

    total_frames = 0

    try:

        #while total_frames < DURATION_TICKS:
        while True:
            node.get_logger().info(">>> Tick...".format(SKIP_TICKS)) 

            _ = world.tick()

            total_frames += 1

    except Exception as e:
        node.get_logger().warn(e)

    finally:

        if original_settings:
            world.apply_settings(original_settings)

        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()

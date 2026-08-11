#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import random
import rospy
import sys
import time

sys.path.append('/opt/ue4_ws/PythonAPI/carla/dist/carla-0.9.13-py2.7-linux-x86_64.egg')
sys.path.append('/opt/ue4_ws/PythonAPI/carla')

import carla

# 随机NPC生成器
# python scripts/spawn_random_npc.py _vehicle_count:=20 _walker_count:=10

class RandomNpcSpawner:
    def __init__(self):
        rospy.init_node('random_npc_spawner', anonymous=True)

        self.vehicle_count = rospy.get_param('~vehicle_count', 10)
        self.walker_count = rospy.get_param('~walker_count', 5)
        self.host = rospy.get_param('~host', 'localhost')
        self.port = rospy.get_param('~port', 2000)
        self.tm_port = rospy.get_param('~tm_port', 8001)
        self.enable_walkers = rospy.get_param('~enable_walkers', True)
        self.walker_ai = rospy.get_param('~walker_ai', False)

        self.carla_client = None
        self.carla_world = None
        self.traffic_manager = None
        self.vehicle_blueprints = []
        self.walker_blueprints = []
        self.spawned_vehicles = []
        self.spawned_walkers = []
        self.spawned_controllers = []

        self.connect_carla()
        self.load_blueprints()
        rospy.on_shutdown(self.destroy_all)

    def connect_carla(self):
        try:
            self.carla_client = carla.Client(self.host, self.port)
            self.carla_client.set_timeout(15.0)
            self.carla_world = self.carla_client.get_world()

            self.traffic_manager = self.carla_client.get_trafficmanager(self.tm_port)
            self.traffic_manager.set_global_distance_to_leading_vehicle(2.5)

            rospy.loginfo("Connected to Carla at {}:{} with Traffic Manager on port {}".format(
                self.host, self.port, self.tm_port))
        except Exception as e:
            rospy.logerr("Failed to connect to Carla: {}".format(e))
            self.carla_client = None
            self.carla_world = None
            self.traffic_manager = None

    def load_blueprints(self):
        if not self.carla_world:
            rospy.logwarn("No Carla world available, cannot load blueprints")
            return

        bp_lib = self.carla_world.get_blueprint_library()

        self.vehicle_blueprints = [bp for bp in bp_lib.filter('vehicle.*')
                                   if bp.has_attribute('number_of_wheels')]
        rospy.loginfo("Loaded {} vehicle blueprints".format(len(self.vehicle_blueprints)))

        if self.enable_walkers:
            self.walker_blueprints = [bp for bp in bp_lib.filter('walker.pedestrian.*')]
            rospy.loginfo("Loaded {} walker blueprints".format(len(self.walker_blueprints)))

    def spawn_npc_vehicle(self, count=10):
        if not self.carla_world or not self.vehicle_blueprints:
            rospy.logwarn("Cannot spawn vehicles: world or blueprints unavailable")
            return

        spawn_points = self.carla_world.get_map().get_spawn_points()
        if not spawn_points:
            rospy.logwarn("No spawn points available on the map")
            return

        spawned_count = 0
        for i in range(count):
            if rospy.is_shutdown():
                break

            blueprint = random.choice(self.vehicle_blueprints)
            spawn_point = random.choice(spawn_points)
            spawn_point.location.z += 0.5

            try:
                vehicle = self.carla_world.spawn_actor(blueprint, spawn_point)
                self.spawned_vehicles.append(vehicle)
                spawned_count += 1

                vehicle.set_autopilot(True, self.tm_port)
                rospy.loginfo("Spawned vehicle {}: {} (id={}) with autopilot".format(
                    i + 1, blueprint.id, vehicle.id))

            except Exception as e:
                rospy.logwarn("Failed to spawn vehicle {}: {}".format(i + 1, e))

        rospy.loginfo("Successfully spawned {} out of {} vehicles".format(spawned_count, count))

    def spawn_npc_walker(self, count=5):
        if not self.enable_walkers:
            rospy.loginfo("Walkers are disabled, skipping...")
            return

        if not self.carla_world or not self.walker_blueprints:
            rospy.logwarn("Cannot spawn walkers: world or blueprints unavailable")
            return

        if not self.walker_ai:
            rospy.loginfo("Spawning static walkers (no AI). Set walker_ai:=true to enable walking.")

        bp_lib = self.carla_world.get_blueprint_library()
        spawn_points = self.carla_world.get_map().get_spawn_points()

        spawned_count = 0
        for i in range(count):
            if rospy.is_shutdown():
                break

            blueprint = random.choice(self.walker_blueprints)

            try:
                if spawn_points:
                    spawn_point = random.choice(spawn_points)
                    spawn_point.location.z += 1.0
                    spawn_point.location.x += random.uniform(-5.0, 5.0)
                    spawn_point.location.y += random.uniform(-5.0, 5.0)
                else:
                    spawn_point = carla.Transform()
                    spawn_point.location = carla.Location(0, 0, 1.0)

                walker = self.carla_world.spawn_actor(blueprint, spawn_point)
                self.spawned_walkers.append(walker)
                spawned_count += 1

                if self.walker_ai:
                    try:
                        controller_bp = bp_lib.find('controller.ai.walker')
                        walker_transform = walker.get_transform()
                        controller = self.carla_world.spawn_actor(controller_bp, walker_transform, walker)
                        self.spawned_controllers.append(controller)

                        controller.start()

                        if spawn_points:
                            target_point = random.choice(spawn_points)
                            target_location = target_point.location
                            target_location.z = 0.0
                        else:
                            target_location = carla.Location(10, 10, 0)

                        controller.go_to_location(target_location)
                        controller.set_max_speed(1.5 + random.random())

                        rospy.loginfo("Spawned walker {}: {} (id={}) with AI controller (id={})".format(
                            i + 1, blueprint.id, walker.id, controller.id))

                    except Exception as ctrl_e:
                        rospy.logwarn("Failed to enable AI for walker {}: {}. Spawning as static.".format(i + 1, ctrl_e))

                else:
                    rospy.loginfo("Spawned static walker {}: {} (id={})".format(i + 1, blueprint.id, walker.id))

            except Exception as e:
                rospy.logwarn("Failed to spawn walker {}: {}".format(i + 1, e))

        rospy.loginfo("Successfully spawned {} out of {} walkers".format(spawned_count, count))

    def destroy_all(self):
        rospy.loginfo("Destroying {} vehicles, {} walkers, {} controllers...".format(
            len(self.spawned_vehicles), len(self.spawned_walkers), len(self.spawned_controllers)))

        for controller in self.spawned_controllers:
            try:
                controller.stop()
                controller.destroy()
            except Exception as e:
                rospy.logwarn("Failed to destroy controller {}: {}".format(controller.id, e))

        for walker in self.spawned_walkers:
            try:
                walker.destroy()
            except Exception as e:
                rospy.logwarn("Failed to destroy walker {}: {}".format(walker.id, e))

        for vehicle in self.spawned_vehicles:
            try:
                vehicle.destroy()
            except Exception as e:
                rospy.logwarn("Failed to destroy vehicle {}: {}".format(vehicle.id, e))

        self.spawned_vehicles = []
        self.spawned_walkers = []
        self.spawned_controllers = []
        rospy.loginfo("All NPCs destroyed")


def main():
    try:
        spawner = RandomNpcSpawner()

        if not spawner.carla_world:
            rospy.logerr("Cannot proceed without Carla world connection")
            return

        rospy.loginfo("Starting to spawn {} vehicles{}...".format(
            spawner.vehicle_count, " and {} walkers".format(spawner.walker_count) if spawner.enable_walkers else ""))

        spawner.spawn_npc_vehicle(spawner.vehicle_count)

        if spawner.enable_walkers and spawner.walker_count > 0:
            spawner.spawn_npc_walker(spawner.walker_count)

        rospy.loginfo("NPC spawning completed! Node will stay alive. Press Ctrl+C to clean up.")

        rospy.spin()

    except rospy.ROSException as e:
        rospy.logerr("ROS exception: {}".format(e))
    except KeyboardInterrupt:
        rospy.loginfo("Spawning interrupted by user")


if __name__ == '__main__':
    main()

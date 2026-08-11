#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
import time
import random

sys.path.append('/opt/ue4_ws/PythonAPI/carla/dist/carla-0.9.13-py2.7-linux-x86_64.egg')
sys.path.append('/opt/ue4_ws/PythonAPI/carla')

import carla


def main():
    host = 'localhost'
    port = 2000
    walker_count = 3
    use_controller = False

    try:
        print("Connecting to Carla at {}:{}...".format(host, port))
        client = carla.Client(host, port)
        client.set_timeout(15.0)
        world = client.get_world()

        print("Loading blueprints...")
        bp_lib = world.get_blueprint_library()

        walker_bps = [bp for bp in bp_lib.filter('walker.pedestrian.*')]
        print("Found {} walker blueprints".format(len(walker_bps)))

        if not walker_bps:
            print("No walker blueprints found")
            return

        spawn_points = world.get_map().get_spawn_points()
        print("Found {} spawn points".format(len(spawn_points)))

        spawned_walkers = []

        for i in range(walker_count):
            print("\nSpawning walker {}...".format(i + 1))

            blueprint = random.choice(walker_bps)

            if spawn_points:
                spawn_point = random.choice(spawn_points)
                spawn_point.location.z += 1.0
                spawn_point.location.x += random.uniform(-5.0, 5.0)
                spawn_point.location.y += random.uniform(-5.0, 5.0)
            else:
                spawn_point = carla.Transform()
                spawn_point.location = carla.Location(0, 0, 1.0)

            print("Spawn point: {}".format(spawn_point.location))

            walker = world.spawn_actor(blueprint, spawn_point)
            spawned_walkers.append(walker)
            print("Spawned walker with id: {}".format(walker.id))

            if use_controller:
                print("Finding controller blueprint...")
                controller_bp = bp_lib.find('controller.ai.walker')
                print("Found controller blueprint")

                walker_transform = walker.get_transform()
                controller = world.spawn_actor(controller_bp, walker_transform, walker)
                print("Spawned controller with id: {}".format(controller.id))

                print("Starting controller...")
                controller.start()
                print("Controller started")

                if spawn_points:
                    target_point = random.choice(spawn_points)
                    target_location = target_point.location
                    target_location.z = 0.0
                else:
                    target_location = carla.Location(10, 10, 0)

                controller.go_to_location(target_location)
                print("Set target location: {}".format(target_location))
                controller.set_max_speed(1.5 + random.random())
                print("Set max speed")

            print("Walker {} setup completed".format(i + 1))

        print("\nAll walkers spawned! Press Ctrl+C to exit...")

        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\nCleaning up...")
            for walker in spawned_walkers:
                walker.destroy()
            print("Cleanup done")

    except Exception as e:
        print("Error: {}".format(e))
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

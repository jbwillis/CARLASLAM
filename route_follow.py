#!/usr/bin/env python

# Modified from examples/tutorial.py
# Jacob Willis, 2019

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random
import time


def main():
    actor_list = []

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        new_map = 'Town03'
        print("Changing map to {}".format(new_map))
        world = client.load_world(new_map)
        world.set_weather(carla.WeatherParameters(cloudyness=20.0, sun_azimuth_angle=90, sun_altitude_angle=90))

        spectator = world.get_spectator() # the spectator is the view of the simulator window

        blueprint_library = world.get_blueprint_library()

        bp = blueprint_library.find('vehicle.audi.tt')

        color = '156,52,8'
        bp.set_attribute('color', color)

        # get the map of the world and the waypoint for the starting location
        world_map = world.get_map()
        starting_wpt = world_map.get_waypoint(carla.Location(x=30, y=-8, z=0))

        # spawn the vehicle at the chosen waypoint and add it to the actor list
        vehicle = world.spawn_actor(bp, starting_wpt.transform)
        actor_list.append(vehicle)
        print('created %s' % vehicle.type_id)

        vehicle.set_autopilot(True)

        # move simulation view to center on vehicle, up high
        world.tick()

        world_snapshot = world.wait_for_tick()
        actor_snapshot = world_snapshot.find(vehicle.id)
        spectator_transform = actor_snapshot.get_transform()
        spectator_transform.location += carla.Location(x=0.0, y=0.0, z=100.0) 
        spectator.set_transform(spectator_transform)

        time.sleep(15)

    finally:

        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.')


if __name__ == '__main__':

    main()

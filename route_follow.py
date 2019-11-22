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
from custom_basic_agent import BasicAgent

import random
import time

from VehicleData import VehicleData

LOC_neighborhood_culdesac = [62, 60, 0]
LOC_town_center = [30, -3, 0]
LOC_highway_neighborhood_edge = [240, 130, 0] 
LOC_center_roundabout_north = [20, 0, 0]
LOC_center_roundabout_south = [-20, 0, 0]


ROUTE_neighborhood_highway = [LOC_highway_neighborhood_edge, LOC_neighborhood_culdesac, LOC_highway_neighborhood_edge, LOC_neighborhood_culdesac]
ROUTE_neighborhood_town_center = [LOC_town_center, LOC_neighborhood_culdesac, LOC_town_center, LOC_neighborhood_culdesac, LOC_town_center]

ROUTE_short = [LOC_center_roundabout_north, LOC_center_roundabout_south, LOC_center_roundabout_north]
ROUTE = ROUTE_short

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
        starting_loc = ROUTE[0]
        starting_wpt = world_map.get_waypoint(carla.Location(x=starting_loc[0], y=starting_loc[1], z=starting_loc[2]))

        # spawn the vehicle at the chosen waypoint and add it to the actor list
        vehicle = world.spawn_actor(bp, starting_wpt.transform)
        actor_list.append(vehicle)
        print('created %s' % vehicle.type_id)

        # vehicle.set_autopilot(True)

        # move simulation view to center on vehicle, up high
        world.tick()

        world_snapshot = world.wait_for_tick()
        actor_snapshot = world_snapshot.find(vehicle.id)
        spectator_transform = actor_snapshot.get_transform()
        spectator_transform.location += carla.Location(x=0, y=0, z=100.0) 
        spectator_transform.rotation.pitch = -89
        spectator_transform.rotation.yaw = -179 
        spectator_transform.rotation.roll = -1 
        spectator.set_transform(spectator_transform)

        pc = vehicle.get_physics_control();
        # get rid of the steering curve
        pc.steering_curve = [carla.Vector2D(0.0, 1.0), carla.Vector2D(120.0, 1.0)];
        vehicle.apply_physics_control(pc)

        # create a basic agent of the vehicle
        agent = BasicAgent(vehicle, target_speed =40)
        agent.set_destination_list(ROUTE)

        # keep track of Vehicle State information
        vd = VehicleData()

        # drive to waypoints until they are all gone
        while len(agent._local_planner._waypoints_queue) > 0:
            world.wait_for_tick(10.0)

            control = agent.run_step()
            control.manual_gear_shift = False
            vehicle.apply_control(control)

            vd.appendVelocityData(vehicle.get_velocity())
            vd.appendControlData(control)
            vd.appendPositionTruth(vehicle.get_location())

        print('Finished following waypoints')


    finally:

        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.')


if __name__ == '__main__':

    main()

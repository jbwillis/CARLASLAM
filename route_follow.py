#!/usr/bin/env python3

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
import pdb
import time
import numpy as np

from VehicleData import VehicleData

# set up an argument parser
import argparse
parser = argparse.ArgumentParser(description='Follow a route in the CARLA simulation')

parser.add_argument('-p', '--plot', help='plot the vehicle data', action='store_true')
parser.add_argument('-vd', '--vehicle_data_file', 
        help='The filename to save VehicleData to')
parser.add_argument('-r', '--route', 
        help='The route # to use. 1 = Roundabout, 2 = Neighborhood and Town Center, 3 = Highway and Neighborhood. Default 1', default=1, type=int)
parser.add_argument('-i', '--interact', help='End with an interactive prompt', action='store_true')

args = parser.parse_args()

LOC_neighborhood_culdesac = [62, 60, 0]
LOC_town_center = [30, -3, 0]
LOC_highway_neighborhood_edge = [240, 130, 0] 
LOC_center_roundabout_north = [20, 0, 0]
LOC_center_roundabout_south = [-20, 0, 0]


ROUTE_neighborhood_highway = [LOC_highway_neighborhood_edge, LOC_neighborhood_culdesac, LOC_highway_neighborhood_edge]
ROUTE_neighborhood_town_center = [LOC_town_center, LOC_neighborhood_culdesac, LOC_town_center]

ROUTE_short = [LOC_center_roundabout_north, LOC_center_roundabout_south, LOC_center_roundabout_north]
routes = [ROUTE_short, ROUTE_neighborhood_town_center, ROUTE_neighborhood_highway]
ROUTE = routes[args.route-1]

def handle_lidar(data):
    point_cloud = np.frombuffer(data.raw_data, dtype=np.float32).reshape([-1, 3])
    point_cloud = np.copy(point_cloud)
    return point_cloud


def add_lidar_sensor(world, vehicle, vd):
    blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    # Modify the attributes of the blueprint to set image resolution and field of view.
    blueprint.set_attribute('range', '5000')

    # Set the time in seconds between sensor captures
    blueprint.set_attribute('sensor_tick', '0.05')
    blueprint.set_attribute('rotation_frequency', '20.0')
    # Provide the position of the sensor relative to the vehicle.
    transform = carla.Transform(carla.Location(x=0.0, z=2.3))
    # Tell the world to spawn the sensor, don't forget to attach it to your vehicle actor.
    sensor = world.spawn_actor(blueprint, transform, attach_to=vehicle)
    sensor.listen(lambda data: vd.appendLidarData(handle_lidar(data)))

    return sensor

def main():
    actor_list = []

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        new_map = 'Town03'
        print("Changing map to {}".format(new_map))
        world = client.load_world(new_map)
        world.set_weather(carla.WeatherParameters(cloudyness=20.0, sun_azimuth_angle=90, sun_altitude_angle=90))

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

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
        vd = VehicleData()
        lidar = add_lidar_sensor(world, vehicle, vd)
        actor_list.append(lidar)
        # vehicle.set_autopilot(True)

        # move simulation view to center on vehicle, up high
        world.tick()

        world_snapshot = world.get_snapshot()
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

        # wheel positions are in global coordinates and in centimeters, convert to wheelbase
        wheel_xy = np.array([(pc.wheels[0].position.x - pc.wheels[2].position.x), (pc.wheels[0].position.y - pc.wheels[2].position.y)])
        wheelbase = 1e-2*np.linalg.norm(wheel_xy)

        # keep track of Vehicle State information
        max_steer = pc.wheels[0].max_steer_angle
        vd.config(wheelbase, max_steer)


        # create a basic agent of the vehicle
        agent = BasicAgent(vehicle, target_speed=40)
        agent.set_destination_list(ROUTE)


        # drive to waypoints until they are all gone
        while len(agent._local_planner._waypoints_queue) > 0:
        # for i in range(150):
            # print(i)
            # world.wait_for_tick(10.00)
            world.tick()
            snapshot = world.get_snapshot()
            time = snapshot.elapsed_seconds

            control = agent.run_step()
            control.manual_gear_shift = False
            vehicle.apply_control(control)
            vd.appendTime(time)
            vd.appendVelocityData(vehicle.get_velocity())
            vd.appendControlData(control)
            vd.appendTransformTruth(vehicle.get_transform())

        control.brake = 1.0
        control.throttle = 0.0
        vehicle.apply_control(control)
        world.tick()


        if args.plot:
            vd.runMotionModelFull()
            vd.plot()

        if args.vehicle_data_file is not None:
            vd.saveToFile(args.vehicle_data_file)
            print("Saved to {}".format(args.vehicle_data_file))

        print('Finished following waypoints')

        # END - Take the simulation out of synchronous/fixed time mode
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        if args.interact:
            import code
            code.interact(local=locals())

    finally:

        print('destroying actors')
        for actor in actor_list:
            if actor.is_alive:
                actor.destroy()
        print('done.')


if __name__ == '__main__':

    main()

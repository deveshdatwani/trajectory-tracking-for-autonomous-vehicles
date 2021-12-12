#!/usr/bin/env python

import carla
import time

import numpy as np

class Car:
    def __init__(self, world, blueprint, transform):

        # actor attributes
        self._world     = world
        self._blueprint = blueprint
        self._transform = transform
        self._actor     = self._world.spawn_actor(self._blueprint, self._transform)

        # physical state
        self.acceleration = self._actor.get_acceleration()
        self.location     = self._actor.get_location()
        self.transform    = self._actor.get_transform()
        self.velocity     = self._actor.get_velocity()
        
        # control parameters
        self.rate = 100 # Hz
        self.kp   = 0.0
        self.kd   = 0.0
    
    def delete(self):

        self._actor.destroy()

    def applyControl(self, throttle, steering, brake):

        # carla.VehicleControl()
        # throttle
        # steer
        # brake
        # hand_brake
        # reverse
        # manual_gear_shift
        # gear
        self._actor.apply_control(carla.VehicleControl(throttle=throttle, steer=steering, brake=brake))
    
    def spinControlLoop(self):
        pass

def main():

    # create client
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    # create world and vehicle
    world = client.get_world()
    blueprintLibrary = world.get_blueprint_library()

    bpVehicle = blueprintLibrary.filter('cybertruck')[0]
    tfVehicle = world.get_map().get_spawn_points()[0] # need to find a better spawn point

    vehicle = Car(world, bpVehicle, tfVehicle)

    # test vehicle control
    throttle = 0.2
    steer    = 0

    direction = -1
    slolam    = 0

    # get up to speed
    vehicle.applyControl(throttle, steer, 0)
    time.sleep(3)

    # swerve around a bit
    while slolam < 3:

        steer += direction*0.1

        if steer <= -1.0:
            direction *= -1

        elif steer >= 1.0:
            direction *= -1
            slolam += 1
        
        vehicle.applyControl(throttle, steer, 0)
        time.sleep(0.1)

    # stop
    vehicle.applyControl(0, steer, 0.5)
    time.sleep(3)

    vehicle.delete()

if __name__ == '__main__':
    
    main()
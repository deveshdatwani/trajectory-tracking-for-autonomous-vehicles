#!/usr/bin/env python

from xmlrpc.client import NOT_WELLFORMED_ERROR
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

        # current state
        self.acceleration = carla.Vector3D()
        self.location     = carla.Location()
        self.transform    = carla.Transform()
        self.velocity     = carla.Vector3D()

        # desired state
        self.accelerationD = carla.Vector3D()
        self.locationD     = carla.Location()
        self.transformD    = carla.Transform()
        self.velocityD     = carla.Vector3D()

        # vehicle control
        self.throttle = 0.0
        self.steer    = 0.0
        self.brake    = 0.0
        
        # controller parameters
        self.kp        = 0.0
        self.kd        = 0.0
        self.kStanley  = 0.8
        self.ksStanley = 0.0
    
    def delete(self):

        self._actor.destroy()
    
    def updatePhysicalState(self):

        self.acceleration = self._actor.get_acceleration()
        self.location     = self._actor.get_location()
        self.transform    = self._actor.get_transform()
        self.velocity     = self._actor.get_velocity()

    def applyControl(self, throttle, steer, brake):

        self._actor.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))
    
    def spinControlLoop1(self):

        # lateral velocity
        # longitudinal velocity
        # yaw rate

        # position
        # yaw

        # u: velocity in longitudinal
        # v: velocity in lattitudinal
        # r: angular velocity
        # theta: heading
        # E: lateral path error

        # xd = Ax + Bu
        
        # u = -kp*e - kd*ed
        pass

    def spinControlLoopStanley(self, pointA, pointB):

        # longitudinal control loop
        # code

        # lateral control loop
        # crosstrack error
        e1 = ((pointB[0] - pointA[0])*(pointA[1] - self.location.y) - (pointA[0] - self.location.x)*(pointB[1] - pointA[1])) / \
            np.sqrt(np.power(pointB[0] - pointA[0], 2) + np.power(pointB[1] - pointA[1], 2))
        
        # orientation error
        e2 = (np.arctan2(pointB[1] - pointA[1], pointB[0] - pointA[0]) - np.deg2rad(self.transform.rotation.yaw))

        # account for full rotations
        if e2 >= np.pi:
            e2 -= 2*np.pi
        elif e2 <= -np.pi:
            e2 += 2*np.pi

        # velocity
        vf = np.linalg.norm((self.velocity.x, self.velocity.y, self.velocity.z))

        # steering angle
        delta = (e2 + np.arctan2(self.kStanley*e1, self.ksStanley + vf))/np.deg2rad(90)

        print('e1:')
        print(e1)
        print('e2:')
        print(np.rad2deg(e2))
        print('delta:')
        print(delta)
        print

        # self.throttle = min(max(u, 0.0), 1.0)
        self.steer = min(max(delta, -1.0), 1.0)

    def printInfo(self):
        print('acceleration:')
        print(self.acceleration)
        print('velocity:')
        print(self.velocity)
        print('location:')
        print(self.location)
        print('rotation:')
        print(self.transform.rotation)
        print

def nearWaypoint(vehicle, waypoint, tolerance = 1.0):

    # verify vehicle is within tolerance of waypoint location
    if abs(waypoint[0] - vehicle.location.x) < tolerance and \
        abs(waypoint[1] - vehicle.location.y) < tolerance:
        return True

    else:
        return False

def main():

    # list of waypoints (x, y, compass)
    # NEED TO INCREASE PRECISION
    waypoints = [(-6.9, -79.1), \
        (-8.0, -25.5), \
        # v return here for circuit
        (-21.9, -10.6), \
        (-40.4, -3.2), \
        (-133.4, -2.6), \
        (-152.6, 17.9), \
        (-152.9, 102.6), \
        (-99.5, 139.9), \
        (124.7, 135.0), \
        (163.3, 117.9), \
        (172.2, 80.8), \
        (187.1, 63.4), \
        (216.2, 62.9), \
        (240.9, 28.8), \
        (246.3, -146.6), \
        (236.8, -187.0), \
        (203.4, -208.3), \
        (101.6, -206.4), \
        (81.7, -179.4), \
        (75.9, -18.9), \
        (61.3, -7.1), \
        (33.0, -7.8), \
        (4.2, -20.2), \
        (-9.9, -19.3)]
    
    # save waypoint goal
    nextWaypoint = 1

    # create client
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    # create world and vehicle
    world = client.get_world()
    blueprintLibrary = world.get_blueprint_library()

    bpVehicle = blueprintLibrary.filter('cybertruck')[0]
    tfVehicle = world.get_map().get_spawn_points()[0]

    vehicle = Car(world, bpVehicle, tfVehicle)

    print('Starting control: waypoint 1')

    # 100Hz frequency
    freq = 100.0

    try:
        while True:

            # save time
            now = time.time()

            # # distance to waypoint
            # dis = np.linalg.norm((waypoints[nextWaypoint][0] - vehicle.location.x, waypoints[nextWaypoint][1] - vehicle.location.y))
            # print('distance to waypoint:')
            # print(dis)

            # increment waypoint if reached
            if nearWaypoint(vehicle, waypoints[nextWaypoint]):

                nextWaypoint += 1
                if nextWaypoint >= len(waypoints):
                    break

                print('Waypoint: ' + str(nextWaypoint))
                print(waypoints[nextWaypoint])
                print

            # do control
            vehicle.updatePhysicalState()
            vehicle.spinControlLoopStanley(waypoints[nextWaypoint - 1], waypoints[nextWaypoint])
            vehicle.applyControl(0.6, vehicle.steer, vehicle.brake)
            # vehicle.printInfo()

            # sleep for fixed frequency
            run = time.time() - now
            time.sleep(1.0/freq - run)

        # # test vehicle control
        # throttle = 0.2
        # steer    = 0

        # direction = -1
        # slolam    = 0

        # # get up to speed
        # vehicle.applyControl(throttle, steer, 0)
        # time.sleep(3)

        # # swerve around a bit
        # while slolam < 3:

        #     steer += direction*0.1

        #     if steer <= -1.0:
        #         direction *= -1

        #     elif steer >= 1.0:
        #         direction *= -1
        #         slolam += 1
            
        #     vehicle.applyControl(throttle, steer, 0)
        #     vehicle.updatePhysicalState()
        #     vehicle.printInfo()
        #     time.sleep(0.1)

        # stop vehicle
        vehicle.applyControl(0.0, 0.0, 0.5)

        # wait a few seconds
        i = 0
        while i <= 3:
            vehicle.updatePhysicalState()
            vehicle.printInfo()
            time.sleep(0.1)
            i += 0.1
    
    finally:
        vehicle.delete()

if __name__ == '__main__':
    
    main()
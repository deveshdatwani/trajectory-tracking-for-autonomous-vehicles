#!/usr/bin/env python

from tkinter import E
from xmlrpc.client import NOT_WELLFORMED_ERROR
import carla
import time
import sys

import numpy as np
import matplotlib.pyplot as plt

class Car:
    def __init__(self, world, blueprint, transform):

        # actor attributes
        self._world     = world
        self._blueprint = blueprint
        self._transform = transform
        self._actor     = self._world.spawn_actor(self._blueprint, self._transform)

        self._MAXSTEERINGANGLE = np.deg2rad(80.0)

        # current state
        self.acceleration = carla.Vector3D()
        self.location     = carla.Location()
        self.transform    = carla.Transform()
        self.velocity     = carla.Vector3D()

        # previous state
        self.accelerationP = carla.Vector3D()
        self.locationP     = carla.Location()
        self.transformP    = carla.Transform()
        self.velocityP     = carla.Vector3D()

        # desired state
        self.speedD   = 0.0
        self.waypoint = carla.Location()

        # extra
        self.eVPrev = 0.0
        self.lastWaypoint = carla.Location()

        # vehicle control
        self.throttle = 0.0
        self.steer    = 0.0
        self.brake    = 0.0
        
        # controller parameters
        self.kpStanley = 1.0
        self.kdStanley = 0.8
        self.kStanley  = 1.0
        self.ksStanley = 0.2
    
    def delete(self):

        self._actor.destroy()
    
    def setSpeed(self, speed):

        self.speedD = speed
    
    def setWaypoint(self, waypoint):

        self.lastWaypoint = self.waypoint
        self.waypoint = carla.Location(waypoint[0], waypoint[1])

    def setPrevWaypoint(self, waypoint):

        self.lastWaypoint = carla.Location(waypoint[0], waypoint[1])
    
    def updatePhysicalState(self):

        self.accelerationP = self.acceleration
        self.locationP     = self.location
        self.transformP    = self.transform
        self.velocityP     = self.velocity

        self.acceleration = self._actor.get_acceleration()
        self.location     = self._actor.get_location()
        self.transform    = self._actor.get_transform()
        self.velocity     = self._actor.get_velocity()

    def applyControl(self, throttle = None, steer = None, brake = None):

        if throttle is None:
            throttle = self.throttle
        if steer is None:
            steer = self.steer
        if brake is None:
            brake = self.brake
        
        self._actor.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))

    def spinControlLoopStanley(self):

        # speed control loop
        # determine speed error (m/s)
        spd  = np.linalg.norm((self.velocity.x, self.velocity.y))
        eV   = self.speedD - spd

        eVdt = eV - self.eVPrev
        self.eVPrev = eV

        # throttle
        u = self.kpStanley*eV + self.kdStanley*eVdt

        # lateral control loop
        # crosstrack error
        e1 = ((self.waypoint.x - self.lastWaypoint.x)*(self.lastWaypoint.y - self.location.y) - \
            (self.lastWaypoint.x - self.location.x)*(self.waypoint.y - self.lastWaypoint.y)) / \
            np.sqrt(np.power(self.waypoint.x - self.lastWaypoint.x, 2) + np.power(self.waypoint.y - self.lastWaypoint.y, 2))
        
        # orientation error
        e2 = np.arctan2(self.waypoint.y - self.lastWaypoint.y, self.waypoint.x - self.lastWaypoint.x) - np.deg2rad(self.transform.rotation.yaw)

        # account for full rotations
        if e2 >= np.pi:
            e2 -= 2*np.pi
        elif e2 <= -np.pi:
            e2 += 2*np.pi

        # steering angle
        delta = (e2 + np.arctan2(self.kStanley*e1, self.ksStanley + spd))/self._MAXSTEERINGANGLE

        # set control
        self.steer    = min(max(delta, -1.0), 1.0)

        if u >= 0.0:
            self.throttle = min(max(u, 0.0), 1.0)
            self.brake    = 0.0
        else:
            self.throttle = 0.0
            self.brake    = min(max(-u, 0.0), 1.0)
        
        # DEBUG
        # err = np.linalg.norm((self.waypoint.x - self.location.x, self.waypoint.y - self.location.y))
        # print('steer:')
        # print(self.steer)
        # print('waypoint error (m):')
        # print(err)
        # print('crosstrack error (e1, m):')
        # print(e1)
        # print('orientation error (e2, rad):')
        # print(e2)
        # print

    def isNearWaypoint(self, tolerance = 3.0):

        # verify vehicle is within tolerance of waypoint location
        if abs(self.waypoint.x - self.location.x) < tolerance and \
            abs(self.waypoint.y - self.location.y) < tolerance:
            return True

        else:
            return False

    def printInfo(self):
        
        print('last waypoint:')
        print(self.lastWaypoint)
        
        print('next waypoint:')
        print(self.waypoint)

        acc = np.linalg.norm((self.acceleration.x, self.acceleration.y))
        print('acceleration (m/s^2):')
        print(acc)

        spd = np.linalg.norm((self.velocity.x, self.velocity.y))
        print('speed (m/s):')
        print(spd)

        print('location (m):')
        print(self.location)

        err = np.linalg.norm((self.waypoint.x - self.location.x, self.waypoint.y - self.location.y))
        print('waypoint error (m):')
        print(err)

        print('heading (deg):')
        print(self.transform.rotation.yaw)
        print


def main(showPlot=True):

    # list of waypoints (x, y) (m)
    waypoints = [(-6.4, -79.1), \
        (-7.2, -35.9),
        (-8.1, -27.1),
        (-11.2, -20.2),
        (-19.4, -9.9),
        (-21.6, 4.4),
        (-16.4, 15.9),
        (-9.8, 30.3),
        (-9.3, 44.3),
        (-8.3, 119.3),
        (-6.1, 127.9),
        (8.0, 133.7),
        (19.7, 135.8),
        (120.5, 135.9),
        (137.1, 133.9),
        (155.2, 125.2),
        (167.4, 109.7),
        (171.2, 85.6),
        (171.7, 72.8),
        (175.5, 64.9),
        (189.0, 61.8),
        (220.0, 61.7),
        (234.8, 57.6),
        (243.1, 39.4),
        (248.0, -139.6),
        (246.1, -162.6),
        (238.7, -183.5),
        (226.8, -197.8),
        (210.6, -207.6),
        (194.5, -208.5),
        (160.3, -205.2),
        (24.4, -205.6),
        (10.5, -204.1),
        (0.1, -195.3),
        (-4.0, -180.7),
        (-6.4, -79.1)]
    
    # remember waypoint goal
    nextWaypoint = 1

    # save vehicle path
    pathX = []
    pathY = []

    # create client
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    # create world and vehicle
    world = client.get_world()
    blueprintLibrary = world.get_blueprint_library()

    bpVehicle = blueprintLibrary.filter('cybertruck')[0]
    tfVehicle = world.get_map().get_spawn_points()[0]

    vehicle = Car(world, bpVehicle, tfVehicle)

    # set up waypoint following
    vehicle.setWaypoint(waypoints[nextWaypoint])
    vehicle.setPrevWaypoint(waypoints[0])
    vehicle.setSpeed(11.176)

    if showPlot:
        fig = plt.figure()
        ax  = fig.add_subplot(111)

        xw = [-p[0] for p in waypoints]
        yw = [p[1] for p in waypoints]

        cir, = ax.plot(xw, yw, linewidth='8', c='k')

        plt.show(False)
        plt.draw()
        plotBG = fig.canvas.copy_from_bbox(ax.bbox)

        pth, = ax.plot(pathX, pathY, c='r')
        wyp, = ax.plot(-vehicle.waypoint.x, vehicle.waypoint.y, '*', c='y')
        car, = ax.plot(-vehicle.location.x, vehicle.location.y, 'o', c='r')

    # start control
    print('Starting control: waypoint 1')
    print(waypoints[nextWaypoint])

    # desired frequency
    freq = 10.0

    try:
        while True:

            # save time
            now = time.time()

            # increment waypoint if reached
            if vehicle.isNearWaypoint():

                nextWaypoint += 1
                if nextWaypoint >= len(waypoints):
                    break

                vehicle.setWaypoint(waypoints[nextWaypoint])

                if showPlot:
                    wyp.set_xdata(-vehicle.waypoint.x)
                    wyp.set_ydata(vehicle.waypoint.y)

                print('Waypoint: ' + str(nextWaypoint))
                print(waypoints[nextWaypoint])
                print

            # do control
            vehicle.updatePhysicalState()
            vehicle.spinControlLoopStanley()
            vehicle.applyControl()
            # vehicle.printInfo()

            # save vehicle location
            pathX.append(-vehicle.location.x)
            pathY.append(vehicle.location.y)
            
            # realtime plot
            if showPlot:
                pth.set_xdata(pathX)
                pth.set_ydata(pathY)
                car.set_xdata(-vehicle.location.x)
                car.set_ydata(vehicle.location.y)

                # redraw plot
                fig.canvas.restore_region(plotBG)
                ax.draw_artist(pth)
                ax.draw_artist(wyp)
                ax.draw_artist(car)
                fig.canvas.blit(ax.bbox)

            # sleep for fixed frequency
            run = time.time() - now

            try:
                time.sleep(1.0/freq - run)
            except:
                slp = 1.0/freq - run
                # if slp < 0:
                #     print('invalid sleep time: ' + str(slp))
                # sys.exit()

        if showPlot:
            # plot path taken
            pth.set_xdata(pathX)
            pth.set_ydata(pathY)

            # redraw plot
            fig.canvas.restore_region(plotBG)
            ax.draw_artist(pth)
            ax.draw_artist(wyp)
            ax.draw_artist(car)
            fig.canvas.blit(ax.bbox)

        # stop vehicle
        vehicle.applyControl(0.0, 0.0, 0.5)
        time.sleep(3)
    
    finally:
        print('Circuit complete')
        vehicle.delete()

if __name__ == '__main__':
    
    main()
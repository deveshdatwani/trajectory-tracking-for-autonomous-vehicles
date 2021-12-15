# Controller_Design
Controller design for autonomous cars. (Worcester Polytechnic Institute RBE-502
Robot Control Final Project by Devesh Datwani and Eric Rogers)

## Controllers
There are two different options for controllers to run the simulation with: a
Stanley controller or a linear feedback controller. These options can be
specified as arguments in the terminal when running the script.

## Setup
Install carla version 0.9.10 (this is the latest version that worked on the
machine used for development)

https://carla.readthedocs.io/en/0.9.10/start_quickstart/

Clone this repository to your local machine

https://github.com/deveshdatwani/control_design

## Running
Run carla

`/opt/carla-simulator/CarlaUE4.sh -opengl`

This will open a window that you can use to explore the simulation. You can use
arrow keys or wasd to fly around and click and hold the mouse to look around. the
`-opengl` option isn't required for all systems, but it is for the machine that
the software was developed on.

Run the control script

`<parent-dir>/control_design/scripts/car_control.py [1|2]`

This will spawn a vehicle in the simulation and drive it in a circuit around the
world using the selected controller (Stanley: 1 or Linear Feedback: 2). A plot
window will also pop up to show the desired path and actual path the vehicle takes
as it drives through each subsequent waypoint.

The user will be prompted by the terminal to `press any key` to start the control
demonstration at the beginning and also at the end after all the plots have been
displayed.

The manual-control script in the examples directory is a modified version of the
example provided by the CARLA simulator install. It is modified to spawn the same
vehicle at the same location as in the control demonstration script.

## Demonstration videos

Stanley Control: https://youtu.be/6bmxXV-_WTk

Linear Feedback Control: https://youtu.be/6x-9eYSEKSo
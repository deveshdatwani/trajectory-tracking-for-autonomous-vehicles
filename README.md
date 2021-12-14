# Controller_Design
Controller design for autonomous cars. (Worcester Polytechnic Institute RBE-502 Robot Control Final Project by Devesh Datwani and Eric Rogers)

## Controller
The Stanley control algorithm is used for lateral control and PID is used for
speed control. The vehicle follows a path defined by sparse waypoints around the
simulated environment and comes to a stop when it reaches its starting location.

## Setup
Install carla version 0.9.10
https://carla.readthedocs.io/en/0.9.10/start_quickstart/

Clone this repository to your local machine
https://github.com/deveshdatwani/control_design

## Running
Run carla
`/opt/carla-simulator/CarlaUE4.sh --opengl`

This will open a window that you can use to explore the simulation. You can use
arrow keys or wasd to fly around and click and hold the mouse to look around.

Run the control script
`<parent-dir>/control_design/scripts/car_control.py`

This will spawn a vehicle in the simulation and drive it in a circuit around the
world. A plot window will also pop up to show the desired path and actual path
the vehicle takes as it drives through each subsequent waypoint.
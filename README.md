
# Solving a Maze Escape Game using Q-Learning
This was built as a final project for CS 5335 - Robotic Science and Systems at Northeastern University.  We reference this paper in our design: [Effective Reinforcement Learning for Mobile Robots](http://www.sci.brooklyn.cuny.edu/~sklar/teaching/f06/air/papers/smart-icra2002.pdf). 

## Game Design
A "prisoner" robot must try to collect a number of keys hidden throughout a maze, then escape the maze without being "caught" by a "patrol" robot.

## Implementation
We attempted to solve this problem first in a simulation environment (designed in Gazebo) and then in a real maze that we built.


This [repo](https://github.com/NatTuck/cs5335hw-gazebo) was referenced for gazebo starter code.


The "prisoner" robot was taught the task of wall following using [q-learning](https://en.wikipedia.org/wiki/Q-learning) in order to navigate the maze.  We implemented q-learning from scratch for this task.  States were considered to be distance intervals on the front and right sensors.  Actions were considered to be velocities for each wheel on the robot.  We designed the maze such that the interior walls that contained the keys were navigatable via wall following.  The prisoner robot was equipped with two ultrasonic sensors (one facing forward and one 90 degrees to the right) and a line following sensor.  The ultrasonic sensors were used to measure distances to the front and side.  The line following sensor was used to locate keys beneath the robot.


The "patrol" robot used a line following sensor to follow a line around the perimeter of the maze.

## Project Layout
The `brain/` folder contains all relavant code for robot operations.

`brain/rg_brain/rg_brain.ino` controls the physical prisoner robot.
`brain/patrol_bot/patrol_bot.ino` controls the physical patrol robot.

`brain/brain.cc` controls the gazebo version of the robot.

`brain/robot.hh` and `brain/gz_robot.cc` act as an API for gazebo usage in c++.

The `models/`, `plugins/`, and `worlds/` folders all contain code for building a gazebo maze.

## Tools Used
- 2 [makeblock rangers](https://www.makeblock.com/project/ranger)
- Arduino CLI or [IDE](https://arduino-ide.en.softonic.com/)
- [Gazebo](http://gazebosim.org/) 
- Ubuntu 20.04
- C++ compiler
- A maze

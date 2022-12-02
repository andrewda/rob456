# ROB456 Final Project

## Setup

```bash
# Clone the repository
$ git clone https://github.com/andrewda/rob456

# Move into the directory and run catkin_make
$ cd rob456 && catkin_make
```

## Running

### Lab 1

```bash
# Launch the world
$ roslaunch lab1 lab1.launch

# Run the stopper code
$ rosrun lab1 stopper.py
```

### Lab 2

```bash
# Launch the world
$ roslaunch lab2 lab2.launch

# Run the driver code
$ rosrun lab2 driver.py

# Run the waypoint publisher
$ rosrun lab2 send_points.py
```
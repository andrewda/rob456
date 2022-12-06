# ROB456 Final Project

[demo.webm](https://user-images.githubusercontent.com/10191084/205789899-6c2bfac0-4054-4b97-a019-5434673b00d5.webm)

## Setup

```bash
# Clone the repository
$ git clone https://github.com/andrewda/rob456

# Move into the directory and run catkin_make
$ cd rob456 && catkin_make

# For lab3, make sure slam-gmapping is installed
$ sudo apt install ros-noetic-slam-gmapping
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

### Lab 3

```bash
# Launch the world
$ roslaunch lab3 lab3.launch
```
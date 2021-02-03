# Swarm-Robotics-Formation-Control
ROBO@FEUP Final Assignment - Multi-robot formation control with ARGoS simulator.

# Requirements
- argos3 simulator
- cmake

# Clone
```shell
$ cd argos3/src/
$ git clone <link>
```

# Build & Compile
```shell
$ mkdir build && cd build
$ cmake ..
$ make
```

# Run 
```shell
cd argos3/src/Swarm-Robotics-Formation-Control
argos3 -c experiments/<filename>.argos
```

# Examples
- Obstacle avoidance behaviour
<img src="https://github.com/avrocha/Swarm-Robotics-Formation-Control/blob/main/images/test_3_c_3_cut.png" width="400" height="400">
<img src="https://github.com/avrocha/Swarm-Robotics-Formation-Control/blob/main/images/test_3_2_cut.png" width="400" height="400">

- Switching formations behaviour
<img src="https://github.com/avrocha/Swarm-Robotics-Formation-Control/blob/main/images/tunel_1.png" width="400" height="400">
<img src="https://github.com/avrocha/Swarm-Robotics-Formation-Control/blob/main/images/tunel_2.png" width="400" height="400">
<img src="https://github.com/avrocha/Swarm-Robotics-Formation-Control/blob/main/images/tunel_3.png" width="400" height="400">

# PX4 MAVLINK Quadrotor Control and Simulation
This repository contains code and resources for controlling and simulating a quadrotor using the MAVLink protocol. It includes implementations for communication, control algorithms, and simulation environments.
## Dependencies
* Behaviortree_cpp v4
* PX4-Autopilot v1.13.0 (release/1.13)
* jmavsim
* java 11

## Installation
1. Follow the instructions to install PX4-Autopilot from the [official PX4 documentation](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html).
2. Configure JMAVSim by following the instructions in the [JMAVSim repository](https://docs.px4.io/main/en/sim_jmavsim/)
3 . Clone this repository:
```bash
git clone
```
4. Navigate to the repository directory and create a build directory:`
```bash
mkdir build && cd build 
cmake ..
make -j12
```
## Usage
Navigate to the PX4-Autopilot directory and run the following command to start the simulation:
```bash
make px4_sitl_default jmavsim
```

Run QGroundControl to connect to the simulated vehicle.
Wait until the vehicle mode is hovering, then you will be able to arm the vehicle and take off using the following commands:
run the telemetry node from this repository:
```bash
cd build 
./quadrotor_teleoperation ../config/teleop_keyboard.xml
```

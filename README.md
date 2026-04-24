# aubo_py3_driver

Python 3 driver wrapper and motion-control examples for AUBO robots. The project builds a `pybind11` module named `aubo_py3`, then uses `aubo_driver/unified_driver.py` from Python scripts.

## Environment

The project was developed on Ubuntu with ROS. ROS Noetic on Ubuntu 20.04 is the recommended baseline.

Required system packages:

```bash
sudo apt update
sudo apt install -y build-essential cmake python3-dev pybind11-dev
sudo apt install -y python3-numpy python3-opencv python3-matplotlib python3-pykdl
sudo apt install -y ros-noetic-rospy ros-noetic-sensor-msgs ros-noetic-geometry-msgs
sudo apt install -y ros-noetic-tf ros-noetic-cv-bridge ros-noetic-kdl-parser-py
```

Optional packages for specific scripts:

```bash
pip3 install minimalmodbus pyserial
```

Some Pinocchio-based test scripts require `pinocchio`. Install it only if you need those scripts.

## Clone

```bash
git clone https://github.com/lmith205-hash/aubo_py3_driver.git
cd aubo_py3_driver
```

## Build

Load ROS first:

```bash
source /opt/ros/noetic/setup.bash
```

Build the Python extension:

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
cd ..
```

After compilation, the extension should be generated as:

```text
aubo_driver/aubo_py3*.so
```

Before running scripts, add the driver directory to `PYTHONPATH`:

```bash
export PYTHONPATH=$PWD/aubo_driver:$PYTHONPATH
```

If the runtime cannot find AUBO shared libraries, also set:

```bash
export LD_LIBRARY_PATH=$PWD/lib/lib64/aubocontroller:$PWD/lib/lib64/config:$PWD/lib/lib64/log4cplus:$LD_LIBRARY_PATH
```

## Configuration

Open the script you want to run and check these values:

```python
USE_SIMULATION = True   # True: simulation, False: real robot
REAL_IP = "100.100.1.10"
URDF_PATH = "/path/to/aubo_i12.urdf"
```

For real robot usage:

- Set `USE_SIMULATION = False`.
- Change `REAL_IP` to your robot controller IP.
- Make sure the robot, controller, and computer are on the same network.

For simulation:

- Set `USE_SIMULATION = True`.
- Make sure the required ROS topics, joint states, and URDF path match your simulation setup.

## Run

Example:

```bash
source /opt/ros/noetic/setup.bash
export PYTHONPATH=$PWD/aubo_driver:$PYTHONPATH
export LD_LIBRARY_PATH=$PWD/lib/lib64/aubocontroller:$PWD/lib/lib64/config:$PWD/lib/lib64/log4cplus:$LD_LIBRARY_PATH
python3 simple_linear_motion.py
```

Other available scripts include:

```text
admittance_control_unified.py
constant_force_control.py
hybrid_motion_force.py
interactive_force_control.py
stable_hybrid_motion.py
aubo_joint_publisher.py
```

## Notes

- This repository contains SDK headers and shared libraries used by the wrapper. Check the AUBO SDK license before redistributing or using them in a public project.
- Several scripts contain local absolute paths such as URDF paths. Update them for your machine before running.
- Test and experiment scripts may be ignored by `.gitignore` and are not required for the basic driver build.

## Troubleshooting

If Python reports `No module named unified_driver` or `No module named aubo_py3`, run:

```bash
export PYTHONPATH=$PWD/aubo_driver:$PYTHONPATH
```

If Python reports missing `.so` libraries, run:

```bash
export LD_LIBRARY_PATH=$PWD/lib/lib64/aubocontroller:$PWD/lib/lib64/config:$PWD/lib/lib64/log4cplus:$LD_LIBRARY_PATH
```

If KDL or ROS imports fail, confirm ROS is installed and sourced:

```bash
source /opt/ros/noetic/setup.bash
```

# autoware_carla_bridge

> [!WARNING]  
> This repo is under development mean that some features may not work as expected.


## Installation

1. Clone the repository into your ROS2 workspace src directory.

``` bash
cd ~/av_ws/src
git clone https://github.com/peeradonmoke2002/autoware_carla_bridge.git
```
2. Install dependencies
```bash
sudo apt update && sudo apt upgrade
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
3. Build the workspace
``` bash
cd ~/av_ws
colcon build --symlink-install
```

## Usage
> [!IMPORTANT]
> Ensure you have installed the CARLA server.
> If not installed, follow the [CARLA installation guide](https://github.com/peeradonmoke2002/Carlar_install.git) at branch ue4.

1. Launch CARLA simulator on your machine or another machine.

``` bash
cd carla/Dist/CARLA_Shipping_0.9.15-330-gdc9e2976d-dirty/LinuxNoEditor
./CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen
```
 
2. Launch `bring_up_carla.launch.py`

Before launching, you need to set the carla host ip and port to match your CARLA simulator settings. First go to file `bring_up_carla.launch.py` and change the default value of `host` and `port` and `town` that your want arguments if needed.

``` bash
cd launch
code bring_up_carla.launch.py # or use your favorite editor
```
edit the default_value of `host`, `port` and `town` arguments if needed.

```python
launch_arguments = {
    'use_sim_time': 'True',
    'host': '10.61.2.24',  # Change this to your CARLA simulator IP address
    'port': '2000',        # Change this to your CARLA simulator port number
    'timeout': '10.0',
    'synchronous_mode': 'True',
    'town': 'Town01',      # Change this to your CARLA simulator town
}
```
Then launch the file using `ros2 launch` command.

``` bash
ros2 launch autoware_carla_bridge bring_up.launch.py
```

3. Launch `autoware_carlar_bridge.launch.py`

```bash
ros2 launch autoware_carla_bridge autoware_carlar_bridge.launch.py
```


![rviz_autoware](./images/rviz_autoware.png)
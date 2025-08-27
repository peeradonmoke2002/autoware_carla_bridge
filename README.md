# autoware_carla_bridge

## Installation

Comming soon...

## Usage

1. Launch CARLA simulator on your machine or another machine.

``` bash
cd carla/Dist/CARLA_Shipping_0.9.15-330-gdc9e2976d-dirty/LinuxNoEditor
./CarlaUE4.sh -prefernvidia -quality-level=Low -nosound -RenderOffScreen
```
 
2. Launch `bring_up.launch.py`

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
Optionally, you can change the default RViz configuration file in the same file.

```python
    rviz_file = os.path.join(
        get_package_share_directory(pkg),
        'rviz','autoware_view.rviz' # Change this to your desired RViz config file name
    )

    # ld.add_action(rviz)   # Uncomment this line to launch RViz automatically 
```


Then launch the file using `ros2 launch` command.

``` bash
ros2 launch autoware_carla_bridge bring_up.launch.py
```

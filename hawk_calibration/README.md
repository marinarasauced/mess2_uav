# hawk_calibration

## Overview
The `hawk_calibration` package provides a method for generating the `calibration.yaml` files for ACE Lab Hawk reconfigurable aerial sensors in ROS2. It consists of an action server and client for collecting measurements from the VICON motion capture system and writing a calibration quaternion. The `calibration.yaml` file is automatically written to `mess2/agents/AGENT_NAME/calibration.yaml`.

**Note:**

Since the `vicon_driver` reads `mess2/agents/` for `calibration.yaml` files statically, before calibrating, **you MUST delete the `calibration.yaml` file for the current AGENT_NAME**.

## Package Contents
- **`server`**: a node that contains an action server for collecting measurements and finding the calibration quaternion.
- **`client`**: an action client node that requests a calibration, and once the result is received, writes the calibration quaternion to `mess2/agents/AGENT_NAME/calibration.yaml`.
- **`client_fake`**: an action client node that publishes fake VICON localization data and requests a calibration, where once the result is received, writes the calibration quaternion to `mess2/agents/AGENT_NAME/calibration.yaml`.
- **`mess2.launch.py`**: a ros2 launch file that starts the server and client on the ground control station.
- **`fake.launch.py`**: a ros2 launch file that starts the server and fake client on the ground control station.

## Installation
The installation of this package is performed during agent setup.

## Manual Usage
**Before running the calibration launch files, delete the `calibration.yaml` file for AGENT_NAME in `mess2/agents/AGENT_NAME/`. By deleting the file, the `vicon_driver` will asumme a unit quaternion making the topic named `/vicon/AGENT_NAME/AGENT_NAME` unaffected**

0. Delete the `calibration.yaml` file for AGENT_NAME.

1. Open a terminal on the GCS and start the `vicon_driver`. Ensure that the AGENT_NAME is actively being published with `ros2 topic list`. If the topic named `/vicon/AGENT_NAME/AGENT_NAME` does not appear, check VICON Tracker to see if AGENT_NAME is being tracked.

```
ros2 launch vicon_receiver client.launch.py
```

2. Place the Hawk in its starting position.
3. Open a new terminal on the GCS and launch the server and client. The server will prompt you to enter a command in another terminal to start sampling data at both positions. The parameter `agent_name` is required, but the parameter `num_measurements` is optional with a default of 1000.

```
ros2 launch hawk_calibration mess2.launch.py agent_name:=AGENT_NAME num_measurements:=1000
```

4. Follow the instructions in the terminal. Move the Hawk and execute the specified commands as necessary.

5. Once the calibration is complete, check `mess2/agents/AGENT_NAME/` for the new `calibration.yaml` file.

6. Enter `CTRL-C` in the terminals running the calibration nodes and the `vicon_driver`. Since the `vicon_driver` reads the `calibration.yaml` files at launch statically, it must be restarted after the calibration in order for changes to take effect.

## Licensing
This package is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
# Deadalus Robotics Peripheral Controller Firmware

## WORK IN PROGRESS!
This is still a work in progress and is not ready to actually be used.

## Setup
### Ubuntu 22.04 or 20.04:
1. Install platformio: [Guide](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html)
2. Install ROS2 [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or [Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
3. Build the firmware and upload it to the feather:
    ```bash
    pio run -t upload
    ```

## Running
You must run [Micro ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent) on the vmc

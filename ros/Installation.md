
# Setting Up ROS 2 Humble on Ubuntu and Raspberry Pi

## Installing ROS 2 Humble

Follow the installation instructions for ROS 2 Humble from the [official documentation](https://docs.ros.org/en/humble/Installation.html).

### Checking the ROS Environment

Verify the ROS environment variables:

```bash
env | grep ROS
```

## Connecting ESP32 via USB

### Checking USB Connection with ESP32

The ESP32 should appear with the command:

```bash
ls /dev/ttyUSB0
```

If not, add the USB device to the virtual box settings.

### Resolving Permission Issues

If you encounter a permission denied error for `/dev/ttyUSB0`, use the following commands to give access to the ESP32:

```bash
sudo lsof /dev/ttyUSB0
```

To check if any program is using the port:

```bash
lsof | grep /dev/ttyUSB0
```

## Connecting Raspberry Pi to Computer via Ethernet

### Computer Configuration (Wired Settings)

1. Go to the IPv4 tab in your network settings.
2. Change to manual.
3. Set the IP address to `192.168.1.1` and netmask to `255.255.255.0`. Gateway is not needed.

### Raspberry Pi Configuration

Edit the Netplan configuration file in `/etc/netplan/` (e.g., `50-cloud-init.yaml` or `01-netcfg.yaml`).

Adjust the configuration:

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.1.2/24
      routes:
        - to: default
          via: 192.168.1.1
```

Apply the configuration (NEED TO BE ADDED TO BASHRC??):

```bash
sudo netplan apply
```

### Testing the Connection

Ping the Ubuntu computer from the Raspberry Pi:

```bash
ping 192.168.1.1
```

Ping the raspberry from the Ubuntu computer 

```bash
ping 192.168.1.2
```

Then ssh to the raspberry :
```bash
ssh pi@192.168.1.2
```
password for pi is raspberry

## Setting Up ROS 2 Humble Environment on Both Devices

### Basic Setup

- Get the IP address of each machine: `hostname -I`.
- Try pinging to check the connection.

### Setting ROS_DOMAIN_ID

Choose a domain ID (0-255) and set it on both devices:

```bash
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
source ~/.bashrc
```

Check the ROS environment variables:

```bash
printenv | grep ROS
```

## Setting Up the Camera on Raspberry Pi

### Enable Camera Hardware

Edit the boot configuration file `/boot/firmware/config.txt`:

```bash
sudo nano /boot/firmware/config.txt
```

Add or uncomment:

```ini
camera_autodetect=0
start_x=1
```

### Install the `ros2_v4l2_camera` Package

Follow the instructions on the [GitHub page](https://github.com/tier4/ros2_v4l2_camera) and [this tutorial](https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304).

Building with `colcon` may take around 1 hour and 10 minutes on a Raspberry Pi 3B+.

Check if a video feed is available:

```bash
ls /dev/video*
```

To launch a node:

```bash
ros2 run v4l2_camera v4l2_camera_node
```

To watch the camera feed:

```bash
ros2 run rqt_image_view rqt_image_view
```

For SSH with GUI forwarding:

```bash
ssh -X pi@IP
```

### Install Compressed Image Transport Plugins

Install necessary plugins for handling compressed image topics:

```bash
sudo apt-get install ros-humble-image-transport-plugins
source /opt/ros/humble/setup.bash
```
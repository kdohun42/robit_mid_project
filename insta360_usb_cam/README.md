# insta360_usb_cam

## udev rule

Fix the Insta360 Link to `/dev/video-insta360`.

```bash
sudo nano /etc/udev/rules.d/99-insta360.rules
```

```udev
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="2e1a", ATTRS{idProduct}=="4c01", ATTR{index}=="0", SYMLINK+="video-insta360"
```

`ATTR{index}=="0"` selects the actual capture node among the multiple `/dev/videoN` nodes registered by UVC.

Apply:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Verify:

```bash
ls -l /dev/video-insta360
```

## Build

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select insta360_usb_cam
source install/setup.bash
```

## Run

```bash
ros2 launch insta360_usb_cam usb_cam.launch.py
```

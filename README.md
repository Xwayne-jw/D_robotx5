# README

```bash
cd ~/ros2_ws
colcon build --packages-select <package-name>
```

```bash
ifconfig
sudo nmcli dev dis wlan0
```


## HDU-203
```bash
192.168.10.168
```

## 启动rosbridge端⼝  --> 上位机costudio
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## 启动底盘
```bash
ros2 launch origincar_base origincar_bringup.launch.py
```

## 启动相机   在studio中更改节点为   /image_compressedw
```bash
ros2 launch origincar_bringup camera.launch.py
```

## 检测中线
```bash
cd racing_ws && source install/setup.bash
ros2 launch racing_track_detection racing_track_detection.launch.py
``
## 障碍物检测
```bash
ros2 launch racing_obstacle_detection obstacle_detection.launch.py
```

## 巡线控制
```bash
cd develop_ws && source install/setup.bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.5 avoid_linear_speed:=0.3 follow_angular_ratio:=-3.0 follow_linear_speed:=0.5 bottom_threshold:=300 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.8
```

## 扫二维码
```bash
ros2 run qr_detector qr_detector_node
```

## 图生文
```bash
ros2 run qwen_pic_to_text qwen_pic_to_text
```

```
sudo hrut_somstatus
```

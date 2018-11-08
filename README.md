# photoneo_test
### Checking the measurement range of photoneo sensors 
- Launch robot in Rviz
```
$ roslaunch vs087_moveit_config demo.launch 
```
- spawn tf with interactive marker  
`$ rosrun tf_interactive_marker.py world [sensor_id]`  
sensor_id: name of tf interactive marker (should be different for each sensor)
```
$ rosrun photoneo_test tf_interactive_marker.py world l1 1.4784 0.0693 0.4358 0.046 0.4315 -3.1227
```
- spawn marker which represents the measurement range of photoneo sensor  
` $ rosrun photoneo_test.py world [sensor_type] [sensor_id]`  
sensor_type: `xs`,`s`,`m`,`l`,`xl`  
sensor_id: same name as tf ineractive merker you spawn above
```
$ rosrun photoneo_test photoneo_test.py l l1
```
- spawn another photoneo sensor with different sensor_id
```
$ rosrun photoneo_test tf_interactive_marker.py world l2 1.4784 0.0693 0.4358 0.046 0.4315 -3.1227
```
```
$ rosrun photoneo_test photoneo_test.py l l2
```
- Rviz config (sample): `photoneo_test/rviz/photoneo_test.rviz`
- information: visit [ARGO CORPORATION's web site](https://www.argocorp.com/cam/special/Photoneo/Photoneo.html)

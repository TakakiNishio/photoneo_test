# photoneo_test
### Checking photoneo sensor's measurement range
```
$ roslaunch vs087_moveit_config demo.launch 
```
```
$ rosrun photoneo_test tf_interactive_marker.py world l1 1.4784 0.0693 0.4358 0.046 0.4315 -3.1227
```
```
$ rosrun photoneo_test photoneo_test.py l l1
```
```
$ rosrun photoneo_test tf_interactive_marker.py world l2 1.4784 0.0693 0.4358 0.046 0.4315 -3.1227
```
```
$ rosrun photoneo_test photoneo_test.py l l2
```

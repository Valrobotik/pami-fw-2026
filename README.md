# PAMI

# Topics

| Topic                          | Type                        | Publishers                        | Subscribers                                                  |
|--------------------------------|-----------------------------|-----------------------------------|--------------------------------------------------------------|
| /goal_pose                     | PoseStamped                 | rviz                              | pami                                                         |
| /pami{n}/position              | PoseStamped                 | pami                              | rviz                                                         |
| /pami{n}/batt                  | Float32                     | pami                              |                                                              |
| /pami{n}/fix_pose              | PoseStamped                 |                                   | pami                                                         |
| /pami{n}/noisette              | Bool                        |                                   | pami                                                         |
| /pami{n}/obstacle              | Bool                        | pami                              |                                                              |

# Running

## MicroROS

### Using docker

```bash
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6
```

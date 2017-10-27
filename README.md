# Map

This package implements a basic map server and provides both a topic publishing [`nav_msgs::OccupancyGrid`](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html) messages and a [`nav_msgs::GetMap`](http://docs.ros.org/api/nav_msgs/html/srv/GetMap.html) service.

## Saving and Loading

Maps can be saved as `.bmp` files using

```
$ rosrun ras_group8_map maputil save -f current_map /ras_group8_map/map
```

To load a saved map, run

```
$ rosrun ras_group8_map maputil load -f current_map /ras_group8_map/update
```


## TODO

- [ ] Load the course maze
- [x] Some method for updating the map
- [ ] Implement rostest tests
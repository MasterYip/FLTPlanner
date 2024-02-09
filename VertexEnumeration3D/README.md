# GCS path search

## Dependencies

- CGAL: `sudo apt install libcgal-dev`

## Examples

### Reachability Analysis

#### Intersection Path Search

![eg_intersect_path.png](doc/eg_intersect_path.png)

```bash
catkin build polyve -DCMAKE_BUILD_TYPE=Release
roslaunch polyve gcs_intersection_search.launch
```

### VisibilityGraph Path Search

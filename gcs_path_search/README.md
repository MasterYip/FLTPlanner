# GCS path search

## Dependencies

- CGAL: `sudo apt install libcgal-dev`

## Build

```bash
catkin build gcs_path_search -DCMAKE_BUILD_TYPE=Release
```

## Examples

### Reachability Analysis

#### GCS Intersection Search

![eg_gcs_intersection_search.png](doc/eg_gcs_intersection_search.png)

```bash
roslaunch gcs_path_search gcs_intersection_search.launch
```

### VisibilityGraph Path Search

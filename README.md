# Jo Description

A comprehensive ROS 2 robot description package for the Jo tracked robot platform, featuring complete robot modeling, and simulated sensors.

<div align="center">
  <img src="res/description.png" alt="Jo Overview" height="500"/>
</div>


## Building
 
```bash
cd ~/ros2_ws
colcon build --packages-select jo_description
source install/setup.bash
```
 
## Usage
 
### Launch only Robot State Publisher
 
```bash
ros2 launch jo_description description.launch.py
```

### Launch with RViz
```bash
ros2 launch jo_description description.launch.py use_rviz:=true
```



## Robot model

The AgileX Bunker Pro 2.0 geometry (body, tracks, track collision shapes) comes from
`AgileX_BUNKER_PRO_2_0.step`. The sensorized platform (rack, lidar, cameras, IMU) is not
in the CAD and is expressed w.r.t. `base_link`.

| Frame            | Where                                                                    |
|------------------|--------------------------------------------------------------------------|
| `base_link`      | top deck of the body (under the rails), longitudinal centre of the upper shell |
| `base_footprint` | ground, 0.454 m below `base_link`                                        |
| `chassis`        | rear top edge of the upper shell                                         |

CAD values are kept in the CAD frame (origin on the ground at the longitudinal centre of
the tracks) and converted with `cad2base_x` / `cad2base_z` in `urdf/robot_core.xacro`.

### Regenerating the meshes

```bash
python3 scripts/step_to_meshes.py AgileX_BUNKER_PRO_2_0.step meshes/
```

Requires `numpy`, `shapely`, `trimesh` and `mapbox_earcut`. The script handles the planar
faceted B-rep exported by this STEP file only.

### Gazebo

Track links (`back_*_roller`, `front_*_roller`, `*_bottom_belt`, `*_back_bottom_belt`,
`*_front_bottom_belt`) are driven by `TrackController`/`TrackedVehicle`. Their frames are
axis-aligned with `base_link`. Meshes use `package://` URIs; the package installs an
environment hook that adds its `share` directory to `GZ_SIM_RESOURCE_PATH`.

## Package Structure
 
```
jo_description/
├── AgileX_BUNKER_PRO_2_0.step              # Robot CAD (source of the meshes)
├── CMakeLists.txt
├── hooks
│   └── jo_description.dsv.in               # GZ_SIM_RESOURCE_PATH hook
├── launch
│   └── description.launch.py
├── meshes                                  # Visual meshes generated from the STEP
├── package.xml
├── README.md
├── res
│   └── description.png
├── rviz
│   └── display.rviz
├── scripts
│   └── step_to_meshes.py                   # STEP -> STL converter
└── urdf
    ├── back_depth.xacro                    # Simulated D455
    ├── front_depth.xacro                   # Simulated D455
    ├── imu.xacro                           # Simulated IMU
    ├── inertial_macros.xacro              
    ├── jo_main.urdf.xacro                  # Main xacro file
    ├── robot_core.xacro                    # Robot's body URDF + rack
    ├── track.xacro                         # Tracks (geometry from CAD) + Gazebo track plugins
    └── velodyne.xacro                      # Simulated lidar

```

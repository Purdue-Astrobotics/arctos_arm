# Physical AprilTag to Gazebo demo

Pipeline: GigE camera (Aravis, Mono8) -> camera/image_raw ->
april_tag_pose_bridge -> /camera/target_pose (world) -> carrot controller ->
MoveIt planning / Servo -> Gazebo. This launch disables the figure-eight source.
Do not run another desired_pose publisher alongside it.

The bridge ports the calibrated 36h11 / SOLVEPNP_IPPE_SQUARE method from
Astro/VA_Test/detect_apriltag.py; it does not import that script because importing
it opens the camera and starts its GUI loop. Calibration comes from the NPZ
keys K, dist, image_size. Images must match calibration resolution and camera
ROI, binning, lens, and focus settings. Tag size is the black square edge in meters.
The selected tag defaults to ID 0, size 0.06 m.

## Setup

Install the workspace ROS dependencies with rosdep and the Aravis 0.8 GI
bindings required by the existing gige_camera_publisher. OpenCV must include
aruco (both legacy and ArucoDetector APIs are supported).

```bash
cd /home/divpr/Developer/Astro/arctos_arm/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Measure the physical camera optical frame relative to the simulated world.
Publish that calibrated transform in another sourced terminal:

```bash
ros2 run tf2_ros static_transform_publisher --x X --y Y --z Z \
  --roll R --pitch P --yaw YAW \
  --frame-id world --child-frame-id physical_camera_optical_frame
```

Replace X/Y/Z with measured meters and R/P/YAW with measured radians. These
are the pose of the optical frame in world, not its inverse. Optical axes are
X right, Y down, Z forward. Do not substitute the camera body frame or an
identity transform unless that alignment is intentional. The bridge publishes
nothing until TF can transform detections into world.

```bash
ros2 launch arctos_control camera_demo.launch.py \
  calibration_path:=/home/divpr/Developer/Astro/VA_Test/calibration/camera_intrinsics_refined.npz \
  tag_id:=0 tag_size:=0.06
```

Optional launch arguments: camera (Aravis ID/IP; empty selects first), rviz,
gz_args. Camera and bridge use simulation time to match Gazebo and the
controller. Gazebo must be running, with its clock advancing.

The pose obeys T_world_target = T_world_camera * T_camera_tag. Tag orientation
is preserved; the optional bridge parameter target_offset is a three-element
translation in tag coordinates (meters), rotated with the tag. Choose an offset
and orientation compatible with arm reachability before tracking. To set this
parameter, run the bridge separately with ROS parameter overrides.

No detections, invalid poses, incorrect image size, or unavailable TF produce
no new targets. The existing carrot controller retains its last target when
updates stop; tag loss does not cancel an active plan or clear that target.

## Validation

Validated Python syntax and ROS Jazzy imports, then exercised the actual bridge
callback with synthetic calibrated image corners. Checked translation and
quaternion recovery, tag ID selection, and resolution rejection. Physical camera
capture and end-to-end Gazebo motion require local hardware and were not tested.

## Camera demo visualization and control

The camera demo now uses /camera/target_pose as its exclusive controller input.
The generic carrot demo still uses /desired_pose. The camera RViz configuration
shows actual robot joints, detected target axes, and /camera/image_with_tags,
without MoveIt's draggable goal or animated plan ghosts. The feed labels
whether the selected ID is visible; detection alone does not confirm a valid
pose or available TF. Watch the bridge tracking log for successful output.

Stop standalone gige_camera_publisher instances and Galaxy Viewer before
launching: only the camera demo should own the camera, using simulation time.

The camera demo defaults to servo_distance:=0.0 (planning only). This avoids
handing unfinished trajectories to Servo near singular configurations. Motion
will be discrete; hold the tag steady while a plan executes. Enable the original
Servo handoff explicitly with servo_distance:=0.25 once target alignment and
reachable poses are verified. Singularity checks remain enabled.

The world origin in this scene is at the floor below table center, not the
tabletop: tabletop Z is 0.75 m and robot base is (0, 0.45, 0.75). A physical
camera 0.40 m above the corresponding tabletop has world Z=1.15 m.

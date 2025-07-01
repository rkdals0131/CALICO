# CALICO Projection Debug Node Usage Guide

## Purpose
The projection debug node helps visualize LiDAR point projections onto camera images to debug sensor fusion issues, particularly when matching returns 0 cones.

## Quick Start

### 1. Build the Package
```bash
cd /home/user1/ROS2_Workspace/ros2_ws
colcon build --packages-select calico
source install/setup.bash
```

### 2. Run the Debug Node

#### Option A: Standalone Debug Node
```bash
# For camera_1
ros2 launch calico projection_debug.launch.py camera_id:=camera_1

# For camera_2 with custom parameters
ros2 launch calico projection_debug.launch.py \
    camera_id:=camera_2 \
    circle_radius:=8 \
    sync_tolerance:=0.2
```

#### Option B: With Full System
```bash
# Enable debug visualization in full launch
ros2 launch calico calico_full.launch.py \
    enable_debug_viz:=true \
    debug_camera_id:=camera_1
```

### 3. View Debug Output

#### Using RQT Image View
```bash
ros2 run rqt_image_view rqt_image_view
# Select topic: /debug/projection_overlay
```

#### Using RViz
Add an Image display and set topic to `/debug/projection_overlay`

## What to Look For

### Visual Elements
- **Green circles**: Projected LiDAR points
- **Yellow text**: Cone index and color (e.g., "0: blue cone")
- **White info bar**: Shows total cones, projected points, and valid projections

### Common Issues and Solutions

1. **No green dots visible**
   - Check if LiDAR data is being published to `/sorted_cones_time`
   - Verify camera calibration parameters are correct
   - Check if points are behind the camera (Z < 0)

2. **Green dots outside image bounds**
   - Camera extrinsic calibration may be incorrect
   - Coordinate system transformation issue

3. **Misaligned projections**
   - Camera intrinsic parameters may be wrong
   - Time synchronization issue between LiDAR and camera

4. **"Time difference too large" warnings**
   - Increase sync_tolerance parameter
   - Check if both LiDAR and camera data are being published

## Debug Information

The node logs detailed information:
```
[INFO] Projection debug: 25 cones -> 18 projected -> 12 valid (in image bounds)
```

This means:
- 25 cones received from LiDAR
- 18 were in front of camera (Z > 0)
- 12 were within image boundaries

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| config_file | Package config | Path to calibration config |
| camera_id | camera_1 | Camera to debug |
| sync_tolerance | 0.1 | Time sync tolerance (seconds) |
| circle_radius | 5 | Size of projected points (pixels) |

## Topics

### Subscribed
- `/sorted_cones_time`: LiDAR cone positions
- `/{camera_id}/image_raw`: Camera image

### Published
- `/debug/projection_overlay`: Annotated image with projections

## Next Steps

Once you can see the projections:
1. Compare projected points with YOLO bounding boxes
2. Check if projections fall within detection boxes
3. Verify the matching distance threshold is appropriate
4. Compare with Python implementation behavior
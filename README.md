# PointCloud Server

**Disclaimer:** This project is a ROS2 wrapper of Kitware's RollingGrid, which is part of their open source [SLAM library](https://gitlab.kitware.com/keu-computervision/slam/-/tree/feat/ROS2?ref_type=heads). The original RollingGrid implementation is licensed under the Apache License 2.0.

## Overview

The PointCloud Server package implements a dynamic, two-level voxel grid for efficient point cloud processing. It wraps Kitware’s RollingGrid and provides:

- **Two-Level Voxel Structure:**
  - **Outer Voxels:** Large, fixed grid cells covering the overall region.
  - **Inner Voxels:** Subdivisions of each outer voxel (controlled by LeafSize) used for downsampling and filtering.

- **Dynamic Grid Management (Rolling):**
  - The grid "rolls" (i.e., shifts its center) so that new scans always fit within the grid.
  - Existing voxels are re-indexed and voxels falling outside the new bounds are discarded.

- **Point Addition and Sampling:**
  - Incoming point clouds are integrated into the grid.
  - Multiple sampling modes are available (first, last, maximum intensity, center point, centroid) to select a representative point per inner voxel.
  - Each inner voxel maintains a count to help distinguish static from transient (moving) points.

- **Submap Extraction:**
  - Extract a submap by merging all points.
  - Extract points within a spatial bounding box, filtering out those observed in too few frames.
  - Extract voxels that intersect a reference point cloud to create a continuous submap.

- **KD-Tree Building:**
  - A KD-tree is built on the submap for fast nearest-neighbor queries.

## Dependencies

- ROS2 (Humble or later)
- PCL (Point Cloud Library)
- Eigen3
- Standard ROS2 packages (e.g., rclcpp, sensor_msgs)

## ROS SERVICES

The node provides several ROS services to interact with the dynamic voxel grid:

- **Add:**  
  Add new points to the grid (with optional grid rolling).

- **BuildSubMap:**  
  Extract a submap using one of several methods (all points, bounding box, or reference point cloud).

- **Clear / ClearPoints:**  
  Clear all points or remove points based on a time threshold.

- **EmptyAroundPoint:**  
  Remove points near a specific location.

- **Get / GetSubMap:**  
  Retrieve the complete grid or the extracted submap as a point cloud.

- **KnnSearch:**  
  Perform a nearest-neighbor search on the submap using a KD-tree.

- **LabelNewPoints:**  
  Label unknown points with respect to the current map.

- **Reset / Roll:**  
  Reset the grid or roll it to a new region.

- **SetGridSize, SetLeafSize, SetVoxelResolution:**  
  Configure grid parameters dynamically.

## License

This project is licensed under the Apache License 2.0.  
See the [LICENSE](LICENSE.txt) file for details.

"""
41068_ignition_bringup package.

This package provides:
- gui_panel.py: Tkinter-based GUI for camera + LiDAR visualization
- filtered_lidar.py: Node to filter LaserScan data
- tree_detector.py: Node to detect trees from LiDAR/vision

The __init__.py file marks this directory as a Python package so ROS2 can find
and install it when building with colcon.
"""

from importlib import metadata

__all__ = [
    "gui_panel",

]

try:
    __version__ = metadata.version("41068_ignition_bringup")
except metadata.PackageNotFoundError:
    __version__ = "unknown"

# gridmap_multipathplanning-main
I write a simple GUI with opencv-python to transfer ROS occupancy map into smaller grid map, and use darp algorithm for coverage path planning.

# Reference
reference to [DARP](https://github.com/alice-st/DARP)

# How to use
1. Install the required packages.
2. Replace the map file with your own
3. Execute the file process.py
4. Slide *gridL* to change each grid length.
5. Slide *inflationR* to limit the free space.
6. Click *x/yBias* to finetune the grids.
6. Double left-clicks to select initial positions for robots, and double right-clicks to delete.
7. Slide *generate* to generate the path.

# gridmap_multipathplanning-main
I write a simple GUI with opencv-python to transfer ROS occupancy map into smaller grid map, and use darp algorithm for coverage path planning.

# Demonstration
1. adjust and align the map(double click on the top map to select robots' places)
![image](https://github.com/jimazeyu/gridmap_multirobot_pathplanning/assets/69748976/dba47758-4729-4fe3-ac28-d89e87754976)

2. generate paths(slide *generate*)
![image](https://github.com/jimazeyu/gridmap_multirobot_pathplanning/assets/69748976/e4c250fd-a53a-49bc-8842-521f6553cb35)


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

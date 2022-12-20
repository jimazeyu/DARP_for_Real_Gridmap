import cv2
import numpy as np
import yaml
from PIL import Image

import DARP.multiRobotPathPlanner as mpp

# variables to adjust
each_grid_len = 10
inflation_radius = 0
x_bias = 0
y_bias = 0

# occupancy map variables
origin_map = 0
occ_occ = 0
occ_unknown = 205
occ_free = 254
map_height = 0
map_width = 0
resolution = 0.05
origin = [] # left-bottom corner of the map

# if visualization is needed
darp_vis = True

# robot position setting
robot_numbers = 0  # robot numbers
robot_portions = []  # robot inspection area portion
robot_place = []  # robot initial position 
obstacles = []

# NULL function
def nothing(x):
    pass

# read grid_map from pgm file
def read_map_frompgm(filename):
    # read config file
    with open(filename+'.yaml') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        global resolution,origin
        resolution = data['resolution']
        origin = data['origin']
    # read img file
    global map_height,map_width,origin_map
    origin_map = Image.open(filename+'.pgm')
    map_height = origin_map.shape[0]
    map_width = origin_map.shape[1]
    # Turn the left-bottom corner to the left-top corner
    origin_map = cv2.flip(origin_map, 0)

# read grid_map from png file
def read_map_frompng(filename):
    # read config file
    with open(filename+'.yaml') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        global resolution,origin
        resolution = data['resolution']
        origin = data['origin']
    # read img file
    global map_height,map_width,origin_map
    origin_map = cv2.imread(filename+'.png')
    origin_map = cv2.cvtColor(origin_map,cv2.COLOR_BGR2GRAY)
    map_height = origin_map.shape[0]
    map_width = origin_map.shape[1]
    # Turn the left-bottom corner to the left-top corner
    origin_map = cv2.flip(origin_map, 0)

# double-click the left button to select, and double-click the right button to delete
def set_robot_place(event, x, y, flags, param):
    global robot_numbers, robot_portions, robot_place
    # double-click the left button to select
    if event == cv2.EVENT_LBUTTONDBLCLK:
        # robot number increase
        robot_numbers += 1
        # set robot portion
        robot_portions = np.ones(robot_numbers)/robot_numbers
        # set robot position
        grid_x = (y-y_bias)//each_grid_len
        grid_y = (x-x_bias)//each_grid_len
        global map_width
        new_pos = grid_x*(map_width//each_grid_len)+grid_y
        robot_place.append(new_pos)
        print("Add robot:", grid_x, grid_y)
    #double-click the right button to delete
    elif event == cv2.EVENT_RBUTTONDBLCLK:
        print("Delete robot")
        robot_numbers = 0
        robot_portions = []
        robot_place = []

# process the map
def process_map():
    global origin_map,obstacles
    obstacles = []
    # copy the origin map
    origin_map_fake = origin_map.copy()

    # inflate the map 
    kernel = np.ones((inflation_radius, inflation_radius), np.uint8)
    inflated_map = cv2.erode(origin_map, kernel, iterations=1)
    
    # convert the map to RGB
    origin_map_fake = cv2.cvtColor(inflated_map, cv2.COLOR_GRAY2BGR)
    processed_map = cv2.cvtColor(inflated_map, cv2.COLOR_GRAY2BGR)

    # draw navigation points
    for i in range(map_height//each_grid_len):
        for j in range(map_width//each_grid_len):
            # calculate the center position in the map
            x = i*each_grid_len + each_grid_len//2 + x_bias
            y = j*each_grid_len + each_grid_len//2 + y_bias
            # judge if the four corners are free
            occ_tag = 0
            for k in range(-1,2,2):
                for l in range(-1,2,2):
                    # calculate the position of the corner
                    x_corner = x + k*each_grid_len//4
                    y_corner = y + l*each_grid_len//4
                    # judge if the corner is in the map
                    if x_corner < 0 or x_corner >= map_height or y_corner < 0 or y_corner >= map_width:
                        occ_tag = 1
                        continue
                    # judge if the corner is free
                    if inflated_map[x_corner][y_corner] != occ_free:
                        occ_tag = 1
            if(occ_tag):
                obstacles.append(i*(map_width//each_grid_len)+j)    
                continue

            # draw the navigation points
            for k in range(-1,2,2):
                for l in range(-1,2,2):
                    # draw the navigation point
                    cv2.circle(origin_map_fake, (y- each_grid_len//4, x- each_grid_len//4), 2, (0, 0, 255), -1)
                    cv2.circle(origin_map_fake, (y- each_grid_len//4, x+ each_grid_len//4), 2, (0, 0, 255), -1)
                    cv2.circle(origin_map_fake, (y+ each_grid_len//4, x- each_grid_len//4), 2, (0, 0, 255), -1)
                    cv2.circle(origin_map_fake, (y+ each_grid_len//4, x+ each_grid_len//4), 2, (0, 0, 255), -1)
                    cv2.rectangle(origin_map_fake, (y - each_grid_len//2, x - each_grid_len//2), (y + each_grid_len//2, x + each_grid_len//2), (255, 0, 0), 1)
                    # draw the center point
                    cv2.circle(processed_map, (y- each_grid_len//4, x- each_grid_len//4), 2, (0, 0, 255), -1)
                    cv2.circle(processed_map, (y- each_grid_len//4, x+ each_grid_len//4), 2, (0, 0, 255), -1)
                    cv2.circle(processed_map, (y+ each_grid_len//4, x- each_grid_len//4), 2, (0, 0, 255), -1)
                    cv2.circle(processed_map, (y+ each_grid_len//4, x+ each_grid_len//4), 2, (0, 0, 255), -1)
            
    # draw the robot position
    for i in range(robot_numbers):
        x_robot = robot_place[i]//(map_width//each_grid_len)
        y_robot = robot_place[i]%(map_width//each_grid_len)
        x_robot = x_robot*each_grid_len + each_grid_len//2 + x_bias
        y_robot = y_robot*each_grid_len + each_grid_len//2 + y_bias
        cv2.circle(origin_map_fake, (y_robot, x_robot), 2, (0, 255, 0), -1)

    # start the window
    processed_map = np.vstack(
        (origin_map_fake, processed_map))
    return processed_map

# generate path
def generate_path(x):
    if(x == 0):
        return
    print("generating path------------------------------")
    global robot_numbers, robot_portions, robot_place, obstacles, map_height, map_width, resolution
    
    '''
    # show the obstacles
    tmp_map = np.zeros((map_height//each_grid_len, map_width//each_grid_len))
    for i in range(len(obstacles)):
        tmp_map[obstacles[i]//(map_width//each_grid_len)][obstacles[i]%(map_width//each_grid_len)] = 1
    # draw the robot position
    for i in range(robot_numbers):
        # calculate the position of the robot
        x_robot = robot_place[i]//(map_width//each_grid_len)
        y_robot = robot_place[i]%(map_width//each_grid_len)
        print(x_robot, y_robot)
        tmp_map[x_robot][y_robot] = 0
    # magnify the map
    tmp_map = cv2.resize(tmp_map, (map_width, map_height), interpolation=cv2.INTER_NEAREST)
    # opencv show
    cv2.imshow("obstacles", tmp_map)
    cv2.waitKey(0)
    '''

    mpp_res = mpp.MultiRobotPathPlanner(
        map_height//each_grid_len, map_width//each_grid_len, True, robot_place,  robot_portions, obstacles, darp_vis)
    path_list = mpp_res.get_best_path()
    # save the path
    for i, path in enumerate(path_list):
        path_name = "./path/path_"+str(i)+".txt"
        f = open(path_name, "w")
        for x1, y1, x2, y2 in path:
            x1 = x1*each_grid_len*resolution/2+each_grid_len*resolution/4+origin[1]+x_bias*resolution
            y1 = y1*each_grid_len*resolution/2+each_grid_len*resolution/4+origin[0]+y_bias*resolution
            x1 = round(x1, 2)
            y1 = round(y1, 2)
            f.write(str(x1)+" "+str(y1)+"\n")
        f.close()
    print("saving path------------------------------")

# GUI loops
def win_loop():
    # control bar
    cv2.namedWindow('Control Bar')
    cv2.resizeWindow('Control Bar', 300, 250)
    cv2.createTrackbar('GridL', 'Control Bar', 25, 50, nothing)
    cv2.createTrackbar('inflationR', 'Control Bar', 1, 50, nothing)
    cv2.createTrackbar('xBias', 'Control Bar', 25, 50, nothing)
    cv2.createTrackbar('yBias', 'Control Bar', 25, 50, nothing)
    # generate the path
    cv2.createTrackbar('generate', 'Control Bar', 0, 1, generate_path)

    # show the map
    cv2.namedWindow('Process the map')
    # click to add robot position
    cv2.setMouseCallback('Process the map', set_robot_place)
    while True:
        global each_grid_len,inflation_radius,x_bias,y_bias
        each_grid_len = cv2.getTrackbarPos('GridL', 'Control Bar')
        inflation_radius = cv2.getTrackbarPos('inflationR', 'Control Bar')
        x_bias = cv2.getTrackbarPos('xBias', 'Control Bar')
        y_bias = cv2.getTrackbarPos('yBias', 'Control Bar')
        # process the map    
        processed_map= process_map()
        # show the map
        cv2.resizeWindow('Process the map', map_width, 2 * map_height)
        cv2.imshow('Process the map', processed_map)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows()

# main
if __name__ == '__main__':
    # read the map
    img = read_map_frompng('./maps/map')

    # GUI loops
    win_loop()
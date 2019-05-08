#!/usr/bin/env python2

import rospy
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


CURR_X = 0
CURR_Y = 0
GOAL_X = 0
GOAL_Y = 0


def meters_to_coord(meters_x, meters_y, map_occ):
    return int(round((meters_x - map_occ.info.origin.position.x +
                      map_occ.info.width * (meters_y -
                                            map_occ.info.origin.position.y)) /
                     map_occ.info.resolution))


def coord_to_meters(coord, map_occ):
#    print('Converting coord: ' + str(coord))
    x, y = float((coord % map_occ.info.width) * map_occ.info.resolution +
               map_occ.info.origin.position.x), float((coord / map_occ.info.width) * map_occ.info.resolution + map_occ.info.origin.position.y)
#    print('x: ' + str(x) + ' y: ' + str(y))
    return x, y


def callback_current(data):
    global CURR_X
    global CURR_Y
    CURR_X = data.pose.pose.position.x
    CURR_Y = data.pose.pose.position.y
#    theta = math.atan2(data.pose.pose.orientation.z,
#                       data.pose.pose.orientation.w)*2
#    print ('Current x: ' + str(x) + ' y: ' + str(y) + ' theta: ' + str(theta))
#    if 'map_obtained' in globals():
#        print ('Current coord: ' + str(meters_to_coord(x, y, map_occ)))


def callback_goal(data):
    global GOAL_X
    global GOAL_Y
    global NEED_PATH
    GOAL_X = data.pose.pose.position.x
    GOAL_Y = data.pose.pose.position.y
    NEED_PATH = True
#   print('Goal x: ' + str(x) + ' y: ' + str(y))
#    if 'map_obtained' in globals():
#        print('Goal coord: ' + str(meters_to_coord(x, y, map_occ)))


def heuristics(cellX, cellY, goal_cellX, goal_cellY):
    return int(abs(cellX - goal_cellX) + abs(cellY - goal_cellY))


def calc_astar(start_x, start_y, goal_x, goal_y, map):
    print('Calculating path beetween: ' + str(start_x) + ' : ' + str(start_y) +
          ' and: ' + str(goal_x) + ' : ' + str(goal_y))
    is_known = []
    is_visited = []
    g_values = []
    f_values = []
    previous = []
    attempts = 0
    for i in range(16000000):
        g_values.append(100000000)
        f_values.append(100000000)
        is_known.append(False)
        is_visited.append(False)
        previous.append(-1)
    print('Initalized...')
    current_cell = meters_to_coord(start_x, start_y, map)
    is_known[current_cell] = True
    g_values[current_cell] = 0
    f_values[current_cell] = 0
    goal_cell = meters_to_coord(goal_x, goal_y, map)
    neighbors = [0, 0, 0, 0]
    visited_cells = []
    while current_cell != goal_cell:
        neighbors[0] = current_cell + 1
        neighbors[1] = current_cell - 1
        neighbors[2] = current_cell + map.info.width
        neighbors[3] = current_cell - map.info.width
#        print('Current cell: ' + str(current_cell))
        for i in range(len(neighbors)):
            if is_known[neighbors[i]]:
                continue
            if map.data[neighbors[i]] > 40 or map.data[neighbors[i]] < 0:
                continue
            g_value = g_values[current_cell] + 1
            goal = meters_to_coord(goal_x, goal_y, map)
            h_value = heuristics(neighbors[i] % 4000, neighbors[i] / 4000,
                                 goal % 4000, goal / 4000)
            if g_value < g_values[neighbors[i]]:
                g_values[neighbors[i]] = g_value
                f_values[neighbors[i]] = g_value + h_value
                previous[neighbors[i]] = current_cell
            if not is_visited[neighbors[i]]:
                visited_cells.append(neighbors[i])
            is_visited[neighbors[i]] = True
        min_idx = -1
        min_f = 100000000
        for i in range(len(visited_cells)):
            if is_known[visited_cells[i]]:
                continue
            if f_values[visited_cells[i]] < min_f:
                min_f = f_values[visited_cells[i]]
                min_idx = visited_cells[i]
        is_known[min_idx] = True
        current_cell = min_idx
        attempts += 1
    print('Exiting the main while...')
    current_cell = goal_cell
    path = Path()
    path.header.frame_id = 'map'
    pose = PoseStamped()
    pose.pose.position.x, pose.pose.position.y = coord_to_meters(current_cell,
                                                                 map)
    pose.pose.position.z = 0
    path.poses.insert(0, pose)
    print('Goal at: ' + str(current_cell))
    print('Calculating path...')
    print(str(previous[current_cell]))
    while previous[current_cell] >= 0:
        #calculate metrics current_cell
        #Path.insert(0,metrics(curren_cell)
        #current_cell = previous[current_cell]
        #print('Cell: ' +str(previous[current_cell]))
        pose = PoseStamped()
        x, y = coord_to_meters(previous[current_cell], map)
        print('Cell: ' + str(previous[current_cell]) + ' x: ' + str(x) +
              ' y: ' + str(y))
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        path.poses.insert(0, pose)
        current_cell = previous[current_cell]
    return path


def maximize_map(map_occ):
    map_max = list(map_occ.data)
    width = map_occ.info.width
    for i in range(len(map_occ.data)):
        x = i % width
        if map_occ.data[i] > 40:
            new_idx = i - 5
            new_idx += width * 5
            for j in range(11):
                for k in range(11):
                    a = (new_idx - j * width) + k
                    if a >= len(map_occ.data) or a < 0:
                        continue
                    if x - k < 0 or x + k >= width:
                        continue
                    map_max[a] = 100

    return map_max


def smooth_path(path):
    b = []
    c = []
    alpha = 0.8
    betha = 0.8
    for i in range(len(path.poses)):
        b.append(0)
        c.append(0)
    b[0] = path.poses[0].pose.position.x
    c[0] = path.poses[0].pose.position.y
    b[len(b) - 1] = path.poses[len(b) - 1].pose.position.x
    c[len(c) - 1] = path.poses[len(c) - 1].pose.position.y
    print('Smoothing the path...')
    err = 1
    while err > 0.01:
        err = 0
        for i in range(1, len(path.poses) - 1):
            b_prev = b[i]
            #b[i] = path.poses[i].pose.position.x + 0.001*(b[i] - path.poses[i].pose.position.x) + 0.5*(path.poses[i+1].pose.position.x + path.poses[i-1].pose.position.x - 2*path.poses[i].pose.position.x)
            b[i] = path.poses[i].pose.position.x + alpha*(b[i] - path.poses[i].pose.position.x) + betha*(b[i+1] + b[i-1] - 2*b[i])
            err += abs(b[i] - b_prev)
        print('ErrX: ' + str(err))
    print('Complete in x')
    err = 1
    while err > 0.01:
        err = 0
        for i in range(1, len(path.poses) - 1):
            c_prev = c[i]
            c[i] = path.poses[i].pose.position.y + alpha*(c[i] - path.poses[i].pose.position.y) + betha*(c[i+1] + c[i-1] - 2*c[i])
            #c[i] = path.poses[i].pose.position.y + 0.001*(c[i] - path.poses[i].pose.position.y) + 0.5*(path.poses[i+1].pose.position.y + path.poses[i-1].pose.position.y - 2*path.poses[i].pose.position.y)
            err += abs(c[i] - c_prev)
        print('ErrY: ' + str(err))
    print('Complete in y')
    smootheth_path = Path()
    smootheth_path.header.frame_id = 'map'
    for i in range(len(b)):
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = b[i], c[i]
        smootheth_path.poses.append(pose)
    print('Complete the smootheth')
    return smootheth_path


def main():
    global CURR_X
    global CURR_Y
    global GOAL_X
    global GOAL_Y
    global NEED_PATH
    NEED_PATH = False
    CURR_X, CURR_Y, GOAL_X, GOAL_Y = 0, 0, 0, 0
    rospy.init_node('astar')
    rate = rospy.Rate(10)
    pub_path = rospy.Publisher('/navigation/path', Path, queue_size=1)
    pub_path_smooth = rospy.Publisher('/navigation/path_smooth', Path, queue_size=1)
    rospy.Subscriber('/navigation/localization/current_pose',
                     PoseWithCovarianceStamped, callback_current)
    rospy.Subscriber('/goal_pose',
                     PoseWithCovarianceStamped, callback_goal)
    clt_getmap = rospy.ServiceProxy('/navigation/localization/static_map',
                                    GetMap)
    rospy.wait_for_service('/navigation/localization/static_map')
    getMapResp = clt_getmap()
    map_occ = getMapResp.map
    print('Ancho: ' + str(map_occ.info.width))
    print('Origen: ' + str(map_occ.info.origin.position.y))
    print('Resolution: ' + str(map_occ.info.resolution))
    print('Last celd is: ' + str(map_occ.data[0]))
    while not rospy.is_shutdown():
        if NEED_PATH:
            map_occ.data = maximize_map(map_occ)
            path = calc_astar(CURR_X, CURR_Y, GOAL_X, GOAL_Y, map_occ)
            smootheth = smooth_path(path)
            pub_path.publish(path)
            pub_path_smooth.publish(smootheth)
            NEED_PATH = False
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

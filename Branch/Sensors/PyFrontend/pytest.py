import struct
import sys
import numpy as np
sys.path.append('./')
sys.path.append('./zzz/')
sys.path.append('./gym_carla/envs/')
from ruleplanner import RulePlanner         
from dynamic_map import DynamicMap
from dynamic_map import Vehicle
from dynamic_map import Lanepoint
from dynamic_map import Lane
from actions import LaneAction



global dynamic_map
dynamic_map = DynamicMap()
global ruleplanner
ruleplanner = RulePlanner()

def func(x1, y1, v1, heading1, vx, vy, x1, y1, v1, heading1, vx, vy):
    data = [0,0]
    return data

def init():
    # update lanes:
    # dynamic_map = DynamicMap()
    # ruleplanner = RulePlanner()
    global dynamic_map
    global ruleplanner
    startx = 0
    starty = 0
    endx = 100
    endy = 0
    t_array = []
    for waypoint in range(0,100):  #FIXME: read from txt: reference path 
        lanepoint = Lanepoint()
        lanepoint.position.x = startx + (endx - startx) * waypoint
        lanepoint.position.y = starty + (endy - starty) * waypoint
        one_lane = Lane()
        one_lane.central_path.append(lanepoint)
        t_array.append(lanepoint)
    one_lane.central_path_array = np.array(t_array)
    one_lane.speed_limit = 30/3.6

    one_lane.lane_index = 0
    dynamic_map.lanes.append(one_lane)
    dynamic_map.lanes_updated = True
    return None

def pnc(x1, y1, v1, heading1, vx, vy):
    global dynamic_map
    global ruleplanner
    obs_num = 5
    target_speed = 30/3.6
    
    if len(ruleplanner.trajectory_planner.frenet_lanes) < 1:
        dynamic_map.lanes_updated = True
    else:
        dynamic_map.lanes_updated = False

    # update dynamic map
    # ego
    dynamic_map.ego_vehicle.x = x1
    dynamic_map.ego_vehicle.y = y1
    dynamic_map.ego_vehicle.v = v1

    dynamic_map.ego_vehicle.yaw = heading1
    # dynamic_map.ego_vehicle.yawdt = ret[4]

    dynamic_map.ego_vehicle.vx = vx
    dynamic_map.ego_vehicle.vy = vy

    # env
    # for i in range(1, obs_num):
    #     vehicle = Vehicle()
    #     vehicle.x = ret[0 + i]
    #     vehicle.y = ret[1 + i]
    #     vehicle.v = ret[2 + i]

    #     vehicle.yaw = ret[3 + i]
    #     vehicle.vx = ret[4 + i]
    #     vehicle.vy = ret[5 + i]
    #     dynamic_map.vehicles.append(vehicle)


    lane_action_RLS = LaneAction(0, target_speed)


    trajectory_action_RLS = ruleplanner.rule_planner_trajectory(dynamic_map, lane_action_RLS)
    print("[ZZZ] : Planning speed",trajectory_action_RLS.desired_speed)

    control_action = ruleplanner.rule_planner_control(dynamic_map, trajectory_action_RLS)
    action = [control_action.acc, control_action.steering]
    print("[ZZZ] : Action_output",action)
    buf1 = control_action.acc
    buf2 = control_action.steering

    data = [buf1, buf2]
    return data


init()
while True:
    pnc(0,0,10,0,10,0)






# test py file, ignore this
# obs_num = 5
# target_speed = 30/3.6
# ruleplanner = RulePlanner()
# dynamic_map = DynamicMap()

# startx = 0
# starty = 0
# endx = 100
# endy = 0
# t_array = []
# for waypoint in range(0,100):  #FIXME: read from txt: reference path 
#     lanepoint = Lanepoint()
#     lanepoint.position.x = startx + (endx - startx) * waypoint
#     lanepoint.position.y = starty + (endy - starty) * waypoint
#     one_lane = Lane()
#     one_lane.central_path.append(lanepoint)
#     t_array.append(lanepoint)
# one_lane.central_path_array = np.array(t_array)
# one_lane.speed_limit = 30/3.6

# one_lane.lane_index = 0
# dynamic_map.lanes.append(one_lane)

# # ego
# dynamic_map.ego_vehicle.x = 0
# dynamic_map.ego_vehicle.y = 0
# dynamic_map.ego_vehicle.v = 10

# dynamic_map.ego_vehicle.yaw = 0
# # dynamic_map.ego_vehicle.yawdt = ret[4]

# dynamic_map.ego_vehicle.vx = 10
# dynamic_map.ego_vehicle.vy = 0
# dynamic_map.lanes_updated = True

# lane_action_RLS = LaneAction(0, target_speed)


# trajectory_action_RLS = ruleplanner.rule_planner_trajectory(dynamic_map, lane_action_RLS)
# print("[ZZZ] : Planning speed",trajectory_action_RLS.desired_speed)

# control_action = ruleplanner.rule_planner_control(dynamic_map, trajectory_action_RLS)
# action = [control_action.acc, control_action.steering]
# print("[ZZZ] : Action_output",action)
# buf1 = control_action.acc
# buf2 = control_action.steering

# data = [buf1, buf2]

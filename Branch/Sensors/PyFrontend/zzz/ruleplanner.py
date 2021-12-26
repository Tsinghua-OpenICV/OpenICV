
import numpy as np
import math

from LanePlanner import LanePlanner
from TrajectoryPlanner import TrajectoryPlanner
from controller import Controller


# def step(dynamic_map):

#     lane_action_rule = ruleplanner.rule_planner_lane(dynamic_map)
#     lane_action_RL = 
#     lane_action_RLS = 

#     trajectory_action_rule = ruleplanner.rule_planner_trajectory(dynamic_map, lane_action_RLS)
#     trajectory_action_RL = 
#     trajectory_action_RLS = 

#     control_action = ruleplanner.rule_planner_control(dynamic_map, trajectory_action_RLS)


class RulePlanner(object):

    def __init__(self):
        self.lane_planner = LanePlanner()
        self.trajectory_planner = TrajectoryPlanner()
        self.controller = Controller()

    def rule_planner_lane(self, dynamic_map):
        return self.lane_planner.run_step(dynamic_map)

    def rule_planner_trajectory(self, dynamic_map, lane_action_RLS):
        return self.trajectory_planner.run_step(dynamic_map,lane_action_RLS.target_lane_index,lane_action_RLS.target_speed)

    def rule_planner_control(self, dynamic_map, trajectory_action_RLS):
        return self.controller.get_control(dynamic_map, 
                            trajectory_action_RLS.trajectory,
                            trajectory_action_RLS.desired_speed,
                            )

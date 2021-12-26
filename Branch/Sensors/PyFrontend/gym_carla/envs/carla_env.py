#!/usr/bin/env python

# Copyright (c) 2019: Jianyu Chen (jianyuchen@berkeley.edu)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import division

import copy
import numpy as np
import pygame
import random
import time
import math
import sys
# import tf

from skimage.transform import resize

import gym
from gym import spaces
from gym.utils import seeding
sys.path.append("/home/icv/ZZZ/carla-098/PythonAPI/carla/dist/carla-0.9.8-py3.5-linux-x86_64.egg")

import carla

from gym_carla.envs.render import BirdeyeRender
from gym_carla.envs.route_planner import RoutePlanner
from gym_carla.envs.misc import *
from gym_carla.envs.dynamic_map import DynamicMap


sys.path.append("/home/icv/ZZZ/carla-098/PythonAPI/carla/")
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
# sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')

from geometry_msgs.msg import PoseStamped, Point32
from carla import Location, Rotation, Transform, Vector3D
from nav_msgs.msg import Path


global first_goal
first_goal = Transform()
first_goal.location.x = 189
first_goal.location.y = -154
first_goal.location.z = 2
first_goal.rotation.pitch = 0
first_goal.rotation.yaw = 0 
first_goal.rotation.roll = 0

global second_goal
second_goal = Transform()
second_goal.location.x = -27
second_goal.location.y = 135
second_goal.location.z = 0
second_goal.rotation.pitch = 0
second_goal.rotation.yaw = 0 
second_goal.rotation.roll = 0

global first_start
first_start = Transform()
first_start.location.x = 206
first_start.location.y = 88
first_start.location.z = 2
first_start.rotation.pitch = 0
first_start.rotation.yaw = -90
first_start.rotation.roll = 0

global second_start
second_start = Transform()
second_start.location.x = -43
second_start.location.y = -2.8
second_start.location.z = 0
second_start.rotation.pitch = 180
second_start.rotation.yaw = 0
second_start.rotation.roll = 0

global stopped_time
stopped_time = np.zeros(1000000)

global has_set
has_set = np.zeros(1000000)

global circle_center
circle_center = carla.Location(-78, -72, 0) # map/circle center

global trajectory_state
trajectory_state = 0

x_max = 250
x_min = 100
y_max = 350
y_min = -320

class CarlaEnv(gym.Env):
  """An OpenAI gym wrapper for CARLA simulator."""

  def __init__(self, params):
    # parameters
    self.display_size = params['display_size']  # rendering screen size
    self.max_past_step = params['max_past_step']
    self.number_of_vehicles = params['number_of_vehicles']
    self.number_of_walkers = params['number_of_walkers']
    self.dt = params['dt']
    self.task_mode = params['task_mode']
    self.max_time_episode = params['max_time_episode']
    self.max_waypt = params['max_waypt']
    self.obs_range = params['obs_range']
    self.lidar_bin = params['lidar_bin']
    self.d_behind = params['d_behind']
    self.obs_size = int(self.obs_range/self.lidar_bin)
    self.out_lane_thres = params['out_lane_thres']
    self.desired_speed = params['desired_speed']
    self.max_ego_spawn_times = params['max_ego_spawn_times']
    self.display_route = params['display_route']
    if 'pixor' in params.keys():
      self.pixor = params['pixor']
      self.pixor_size = params['pixor_size']
    else:
      self.pixor = False

    # Init dynamic_map
    self.dynamic_map = DynamicMap()

    # Destination
    self.dests = [[204, -72, 0]]
   

    # action and observation spaces
    self.discrete = params['discrete']
    self.discrete_act = [params['discrete_acc'], params['discrete_steer']] # acc, steer
    self.n_acc = len(self.discrete_act[0])
    self.n_steer = len(self.discrete_act[1])
    if self.discrete:
      self.action_space = spaces.Discrete(self.n_acc*self.n_steer)
    else:
      self.action_space = spaces.Box(np.array([params['continuous_accel_range'][0], 
      params['continuous_steer_range'][0]]), np.array([params['continuous_accel_range'][1],
      params['continuous_steer_range'][1]]), dtype=np.float32)  # acc, steer
    observation_space_dict = {
      # 'camera': spaces.Box(low=0, high=255, shape=(self.obs_size, self.obs_size, 3), dtype=np.uint8),
      # 'lidar': spaces.Box(low=0, high=255, shape=(self.obs_size, self.obs_size, 3), dtype=np.uint8),
      'birdeye': spaces.Box(low=0, high=255, shape=(3, self.obs_size, self.obs_size), dtype=np.uint8),
      # 'state': spaces.Box(np.array([-2, -1, -5, 0]), np.array([2, 1, 30, 1]), dtype=np.float32)
      }
    if self.pixor:
      observation_space_dict.update({
        'roadmap': spaces.Box(low=0, high=255, shape=(self.obs_size, self.obs_size, 3), dtype=np.uint8),
        'vh_clas': spaces.Box(low=0, high=1, shape=(self.pixor_size, self.pixor_size, 1), dtype=np.float32),
        'vh_regr': spaces.Box(low=-5, high=5, shape=(self.pixor_size, self.pixor_size, 6), dtype=np.float32),
        'pixor_state': spaces.Box(np.array([-1000, -1000, -1, -1, -5]), np.array([1000, 1000, 1, 1, 20]), dtype=np.float32)
        })
    # self.observation_space = spaces.Dict(observation_space_dict)
    self.observation_space = spaces.Box(low=0, high=255, shape=(3, self.obs_size, self.obs_size), dtype=np.uint8)

    # Connect to carla server and get world object
    print('Connecting to Carla server...')
    self.client = carla.Client('localhost', params['port'])
    self.client.set_timeout(10.0)
    self.world = self.client.load_world(params['town'])
    self.tm = self.client.get_trafficmanager(8000)

    print('Carla server connected!')

    # Set weather
    self.world.set_weather(carla.WeatherParameters.ClearNoon)

    # Get spawn points
    self.vehicle_spawn_points = list(self.world.get_map().get_spawn_points())
    # self.walker_spawn_points = []
    # for i in range(self.number_of_walkers):
    #   spawn_point = carla.Transform()
    #   loc = self.world.get_random_location_from_navigation()
    #   if (loc != None):
    #     spawn_point.location = loc
    #     self.walker_spawn_points.append(spawn_point)

    # Create the ego vehicle blueprint
    self.ego_bp = self._create_vehicle_bluepprint(params['ego_vehicle_filter'], color='49,8,8')

    # Collision sensor
    self.collision_hist = [] # The collision history
    self.collision_hist_l = 1 # collision history length
    self.collision_bp = self.world.get_blueprint_library().find('sensor.other.collision')

    # Lidar sensor
    self.lidar_data = None
    self.lidar_height = 2.1
    self.lidar_trans = carla.Transform(carla.Location(x=0.0, z=self.lidar_height))
    self.lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
    self.lidar_bp.set_attribute('channels', '32')
    self.lidar_bp.set_attribute('range', '5000')

    # Camera sensor
    self.camera_img = np.zeros((self.display_size, self.display_size, 3), dtype=np.uint8)
    self.camera_trans = carla.Transform(carla.Location(x=0.8, z=1.7))
    self.camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
    print('Ego_vehicle set!')

    # Modify the attributes of the blueprint to set image resolution and field of view.
    self.camera_bp.set_attribute('image_size_x', str(self.display_size))
    self.camera_bp.set_attribute('image_size_y', str(self.display_size))
    self.camera_bp.set_attribute('fov', '110')
    # Set the time in seconds between sensor captures
    self.camera_bp.set_attribute('sensor_tick', '0.02')

    # Set fixed simulation step for synchronous mode
    self.settings = self.world.get_settings()
    self.settings.fixed_delta_seconds = self.dt

    # Record the time of total steps and resetting steps
    self.reset_step = 0
    self.total_step = 0

    # Initialize the renderer
    time1 = time.time()
    self._init_renderer()
    self.h_reconstruct = None
    self.reconstruct_sample = None
    time2 = time.time()
    print("Initial render time:",time2 - time1)
    
    self.waypoints = []
    self._max_episode_steps = 1000 # Deep Bisim need this

    print("[CARLA_GYM] : Initilize finished")

  def reset(self):
    # Clear sensor objects  
    self.collision_sensor = None
    self.lidar_sensor = None
    self.camera_sensor = None

    # Delete sensors, vehicles and walkers
    self._clear_all_actors(['sensor.other.collision', 'sensor.lidar.ray_cast', 'sensor.camera.rgb', 'vehicle.*', 'controller.ai.walker', 'walker.*'])

    # Disable sync mode
    self._set_synchronous_mode(False)

    # Get actors polygon list
    self.vehicle_polygons = []
    vehicle_poly_dict = self._get_actor_polygons('vehicle.*')
    self.vehicle_polygons.append(vehicle_poly_dict)
    self.walker_polygons = []
    walker_poly_dict = self._get_actor_polygons('walker.*')
    self.walker_polygons.append(walker_poly_dict)

    # Spawn the ego vehicle
    ego_spawn_times = 0
    while True:
      if ego_spawn_times > self.max_ego_spawn_times:
        self.reset()

      if self._try_spawn_ego_vehicle_at(first_start):
        break
      else:
        ego_spawn_times += 1
        time.sleep(0.1)
    
    # Generate Reference Path
    dao = GlobalRoutePlannerDAO(self.world.get_map())
    self.calculate_route(first_start, first_goal, dao)
    


    # Add collision sensor
    self.collision_sensor = self.world.spawn_actor(self.collision_bp, carla.Transform(), attach_to=self.ego)
    self.collision_sensor.listen(lambda event: get_collision_hist(event))
    def get_collision_hist(event):
      impulse = event.normal_impulse
      intensity = np.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
      self.collision_hist.append(intensity)
      if len(self.collision_hist)>self.collision_hist_l:
        self.collision_hist.pop(0)
    self.collision_hist = []

    # Add lidar sensor
    self.lidar_sensor = self.world.spawn_actor(self.lidar_bp, self.lidar_trans, attach_to=self.ego)
    self.lidar_sensor.listen(lambda data: get_lidar_data(data))
    def get_lidar_data(data):
      self.lidar_data = data

    # Add camera sensor
    self.camera_sensor = self.world.spawn_actor(self.camera_bp, self.camera_trans, attach_to=self.ego)
    self.camera_sensor.listen(lambda data: get_camera_img(data))
    def get_camera_img(data):
      array = np.frombuffer(data.raw_data, dtype = np.dtype("uint8"))
      array = np.reshape(array, (data.height, data.width, 4))
      array = array[:, :, :3]
      array = array[:, :, ::-1]
      self.camera_img = array

    # Update timesteps
    self.time_step=0
    self.reset_step+=1

    # Enable sync mode
    self.settings.synchronous_mode = True
    self.world.apply_settings(self.settings)

    # Set ego information for render
    self.birdeye_render.set_hero(self.ego, self.ego.id)

    # Removed stopped vehicles
    self.removed_stopped_vehicle(self.world)

    # Temp: spawn one vehicle
    self.spawn_that_vehicle(self.world)

    return self._get_obs()
  
  def step(self, action):
    # Control traffic
    self.generate_traffic_098(self.world)
    self.removed_too_far_vehicle(self.world)

    # Adjust ref path - Needed in Town05
    # dao = GlobalRoutePlannerDAO(self.world.get_map())
    # self.adjust_ref_path(dao)
   
    # Calculate acceleration and steering
    if self.discrete:
      acc = self.discrete_act[0][action//self.n_steer]
      steer = self.discrete_act[1][action%self.n_steer]
    else:
      acc = action[0]
      steer = action[1]

    # Convert acceleration to throttle and brake
    if acc > 0:
      throttle = acc#np.clip(acc/3,0,1)
      brake = 0
    else:
      throttle = 0
      brake = -acc * 3#np.clip(-acc/8,0,1)
    print("[CARLA_GYM] : Output action", throttle, steer, brake)
    # Apply control
    act = carla.VehicleControl(throttle=float(throttle), steer=float(steer), brake=float(brake))
    self.ego.apply_control(act)

    self.world.tick()

    # Append actors polygon list
    vehicle_poly_dict = self._get_actor_polygons('vehicle.*')
    self.vehicle_polygons.append(vehicle_poly_dict)
    while len(self.vehicle_polygons) > self.max_past_step:
      self.vehicle_polygons.pop(0)
    walker_poly_dict = self._get_actor_polygons('walker.*')
    self.walker_polygons.append(walker_poly_dict)
    while len(self.walker_polygons) > self.max_past_step:
      self.walker_polygons.pop(0)

    # state information
    info = {
      # 'waypoints': self.waypoints,
      # 'vehicle_front': self.vehicle_front
    }
    
    # Update timesteps
    self.time_step += 1
    self.total_step += 1
    return (self._get_obs(), self._get_reward(), self._terminal(), copy.deepcopy(info))

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def render(self, mode):
    pass

  def _create_vehicle_bluepprint(self, actor_filter, color=None, number_of_wheels=[4]):
    """Create the blueprint for a specific actor type.

    Args:
      actor_filter: a string indicating the actor type, e.g, 'vehicle.lincoln*'.

    Returns:
      bp: the blueprint object of carla.
    """
    blueprints = self.world.get_blueprint_library().filter(actor_filter)
    blueprint_library = []
    for nw in number_of_wheels:
      blueprint_library = blueprint_library + [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == nw]
    bp = random.choice(blueprint_library)
    if bp.has_attribute('color'):
      if not color:
        color = random.choice(bp.get_attribute('color').recommended_values)
      bp.set_attribute('color', color)
      bp.set_attribute('role_name',"ego_vehicle")
    return bp

  def _init_renderer(self):
    """Initialize the birdeye view renderer.
    """
    pygame.init()
    self.display = pygame.display.set_mode(
    (self.display_size * 4, self.display_size),
    pygame.HWSURFACE | pygame.DOUBLEBUF)

    pixels_per_meter = self.display_size / self.obs_range
    pixels_ahead_vehicle = (self.obs_range/2 - self.d_behind) * pixels_per_meter
    birdeye_params = {
      'screen_size': [self.display_size, self.display_size],
      'pixels_per_meter': pixels_per_meter,
      'pixels_ahead_vehicle': pixels_ahead_vehicle
    }
    self.birdeye_render = BirdeyeRender(self.world, birdeye_params)

  def _set_synchronous_mode(self, synchronous = True):
    """Set whether to use the synchronous mode.
    """
    self.settings.synchronous_mode = synchronous
    self.world.apply_settings(self.settings)

  def _try_spawn_random_vehicle_at(self, transform, number_of_wheels=[4]):
    """Try to spawn a surrounding vehicle at specific transform with random bluprint.

    Args:
      transform: the carla transform object.

    Returns:
      Bool indicating whether the spawn is successful.
    """
    blueprint = self._create_vehicle_bluepprint('vehicle.*', number_of_wheels=number_of_wheels)
    blueprint.set_attribute('role_name', 'autopilot')
    vehicle = self.world.try_spawn_actor(blueprint, transform)
    if vehicle is not None:
      vehicle.set_autopilot()
      return True
    return False

  def _try_spawn_random_walker_at(self, transform):
    """Try to spawn a walker at specific transform with random bluprint.

    Args:
      transform: the carla transform object.

    Returns:
      Bool indicating whether the spawn is successful.
    """
    walker_bp = random.choice(self.world.get_blueprint_library().filter('walker.*'))
    # set as not invencible
    if walker_bp.has_attribute('is_invincible'):
      walker_bp.set_attribute('is_invincible', 'false')
    walker_actor = self.world.try_spawn_actor(walker_bp, transform)

    if walker_actor is not None:
      walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
      walker_controller_actor = self.world.spawn_actor(walker_controller_bp, carla.Transform(), walker_actor)
      # start walker
      walker_controller_actor.start()
      # set walk to random point
      walker_controller_actor.go_to_location(self.world.get_random_location_from_navigation())
      # random max speed
      walker_controller_actor.set_max_speed(1 + random.random())    # max speed between 1 and 2 (default is 1.4 m/s)
      return True
    return False

  def _try_spawn_ego_vehicle_at(self, transform):
    """Try to spawn the ego vehicle at specific transform.
    Args:
      transform: the carla transform object.
    Returns:
      Bool indicating whether the spawn is successful.
    """
    print("Try spawn ego vehicle")
    vehicle = None
    # Check if ego position overlaps with surrounding vehicles
    overlap = False
    for idx, poly in self.vehicle_polygons[-1].items():
      poly_center = np.mean(poly, axis=0)
      ego_center = np.array([transform.location.x, transform.location.y])
      dis = np.linalg.norm(poly_center - ego_center)
      print("Try spawn ego vehicle2")

      if dis > 8:
        continue
      else:
        overlap = True
        break

    if not overlap:
      vehicle = self.world.try_spawn_actor(self.ego_bp, transform)
      print("Try spawn ego vehicle3")


    if vehicle is not None:
      print("Try spawn ego vehicle4")

      self.ego=vehicle
      return True
      
    return False

  def _get_actor_polygons(self, filt):
    """Get the bounding box polygon of actors.

    Args:
      filt: the filter indicating what type of actors we'll look at.

    Returns:
      actor_poly_dict: a dictionary containing the bounding boxes of specific actors.
    """
    actor_poly_dict={}
    for actor in self.world.get_actors().filter(filt):
      # Get x, y and yaw of the actor
      trans=actor.get_transform()
      x=trans.location.x
      y=trans.location.y
      yaw=trans.rotation.yaw/180*np.pi
      # Get length and width
      bb=actor.bounding_box
      l=bb.extent.x
      w=bb.extent.y
      # Get bounding box polygon in the actor's local coordinate
      poly_local=np.array([[l,w],[l,-w],[-l,-w],[-l,w]]).transpose()
      # Get rotation matrix to transform to global coordinate
      R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
      # Get global bounding box polygon
      poly=np.matmul(R,poly_local).transpose()+np.repeat([[x,y]],4,axis=0)
      actor_poly_dict[actor.id]=poly
    return actor_poly_dict

  def _get_obs(self):
    """Get the observations."""
    ## Birdeye rendering
    self.birdeye_render.vehicle_polygons = self.vehicle_polygons
    self.birdeye_render.walker_polygons = self.walker_polygons  
    self.birdeye_render.waypoints = self.waypoints

    # birdeye view with roadmap and actors
    birdeye_render_types = ['roadmap', 'actors']
    if self.display_route:
      birdeye_render_types.append('waypoints')

    self.birdeye_render.render(self.display, birdeye_render_types)

    birdeye = pygame.surfarray.array3d(self.display)
    birdeye = birdeye[0:self.display_size, :, :]
    birdeye = display_to_rgb(birdeye, self.obs_size)

    # Roadmap
    if self.pixor:
      roadmap_render_types = ['roadmap']
      if self.display_route:
        roadmap_render_types.append('waypoints')
      self.birdeye_render.render(self.display, roadmap_render_types)
      roadmap = pygame.surfarray.array3d(self.display)
      roadmap = roadmap[0:self.display_size, :, :]
      roadmap = display_to_rgb(roadmap, self.obs_size)
      # Add ego vehicle
      for i in range(self.obs_size):
        for j in range(self.obs_size):
          if abs(birdeye[i, j, 0] - 255)<20 and abs(birdeye[i, j, 1] - 0)<20 and abs(birdeye[i, j, 0] - 255)<20:
            roadmap[i, j, :] = birdeye[i, j, :]

    # Display birdeye image
    birdeye_surface = rgb_to_display_surface(birdeye, self.display_size)
    self.display.blit(birdeye_surface, (self.display_size * 1, 0))

    # Display camera image
    camera = resize(self.camera_img, (self.display_size, self.display_size)) * 255
    camera_surface = rgb_to_display_surface(camera, self.display_size)
    self.display.blit(camera_surface, (self.display_size * 0, 0))

    # Display reconstruct and sample image
    if self.h_reconstruct is not None:
      reconstruct = np.transpose(self.h_reconstruct, [1, 2, 0])
      reconstruct = resize(reconstruct, (self.obs_size, self.obs_size)) * 255
      reconstruct_surface = rgb_to_display_surface(reconstruct, self.display_size)
      self.display.blit(reconstruct_surface, (self.display_size * 2, 0))

    if self.reconstruct_sample is not None:
      reconstruct_sample = np.transpose(self.reconstruct_sample, [1, 2, 0])
      reconstruct_sample = resize(reconstruct_sample, (self.obs_size, self.obs_size)) * 255
      reconstruct_sample_surface = rgb_to_display_surface(reconstruct_sample, self.display_size)
      self.display.blit(reconstruct_sample_surface, (self.display_size * 3, 0))
    
    # birdeye = resize(birdeye, (self.obs_size, self.obs_size)) * 255
    birdeye = np.transpose(birdeye, [2, 0, 1])  # 3 x 84 x 84/252/420
    print("[CARLA_GYM]: birdeye.shape",birdeye.shape)

    # update dynamic_map
    self.dynamic_map.update_map(self.world)
    # Display on pygame
    pygame.display.flip()
    obs = {
      'camera':camera.astype(np.uint8),
      'birdeye':birdeye.astype(np.uint8),   # 400*400*3
      'dynamic_map': self.dynamic_map,
    }

    return obs

  def _get_reward(self):
    """Calculate the step reward."""
    # reward for speed tracking
    v = self.ego.get_velocity()
    speed = np.sqrt(v.x**2 + v.y**2)
    r_speed = -abs(speed - self.desired_speed)
    
    # reward for collision
    r_collision = 0
    if len(self.collision_hist) > 0:
      r_collision = -1

    # reward for steering:
    r_steer = -self.ego.get_control().steer**2

    # reward for out of lane
    ego_x, ego_y = get_pos(self.ego)
    dis, w = get_lane_dis(self.waypoints, ego_x, ego_y)
    r_out = 0
    if abs(dis) > self.out_lane_thres:
      r_out = -1

    # longitudinal speed
    lspeed = np.array([v.x, v.y])
    lspeed_lon = np.dot(lspeed, w)

    # cost for too fast
    r_fast = 0
    if lspeed_lon > self.desired_speed:
      r_fast = -1

    # cost for lateral acceleration
    r_lat = - abs(self.ego.get_control().steer) * lspeed_lon**2

    r = 200*r_collision + 1*lspeed_lon + 10*r_fast + 1*r_out + r_steer*5 + 0.2*r_lat - 0.1

    return r

  def _terminal(self):
    """Calculate whether to terminate the current episode."""
    # Get ego state
    ego_x, ego_y = get_pos(self.ego)

    # If collides
    if len(self.collision_hist)>0: 
      return True

    # If reach maximum timestep
    if self.time_step>self.max_time_episode:
      return True

    # If at destination
    if self.dests is not None: # If at destination
      for dest in self.dests:
        if np.sqrt((ego_x-dest[0])**2+(ego_y-dest[1])**2)<20:
          return True

    # If out of lane
    dis, _ = get_lane_dis(self.waypoints, ego_x, ego_y)
    if abs(dis) > self.out_lane_thres:
      return True

    return False

  def _clear_all_actors(self, actor_filters):
    """Clear specific actors."""
    for actor_filter in actor_filters:
      for actor in self.world.get_actors().filter(actor_filter):
        if actor.is_alive:
          if actor.type_id == 'controller.ai.walker':
            actor.stop()
          actor.destroy()

  def calculate_route(self, start, goal, dao):
      print("[CARLA_GYM] : Calculating route to x={}, y={}, z={}".format(
              goal.location.x,
              goal.location.y,
              goal.location.z))
      self.waypoints = []
      grp = GlobalRoutePlanner(dao)
      grp.setup()
      current_route = grp.trace_route(carla.Location(start.location.x,
                                              start.location.y,
                                              start.location.z),
                              carla.Location(goal.location.x,
                                              goal.location.y,
                                                goal.location.z))
      msg = Path()
      msg.header.frame_id = "map"

      for wp in current_route:
          pose = PoseStamped()
          pose.pose.position.x = wp[0].transform.location.x
          pose.pose.position.y = wp[0].transform.location.y
          pose.pose.position.z = wp[0].transform.location.z
          
          # FIXME: the tf library has problem
          # quaternion = tf.transformations.quaternion_from_euler(
          #     0, 0, -math.radians(wp[0].transform.rotation.yaw))
          # pose.pose.orientation.x = quaternion[0]
          # pose.pose.orientation.y = quaternion[1]
          # pose.pose.orientation.z = quaternion[2]
          # pose.pose.orientation.w = quaternion[3]
          msg.poses.append(pose)
          self.waypoints.append([wp[0].transform.location.x, wp[0].transform.location.y, wp[0].transform.rotation.yaw])

      self.dynamic_map.map.setup_reference_lane_list(msg)
      print("[CARLA_GYM] : Published {} waypoints.".format(len(msg.poses)))

  def generate_traffic_098(self, carla_world, delay = 0.05):
    global has_set
    blueprints_ori = carla_world.get_blueprint_library().filter('vehicle.*')
    spawn_points_ori = carla_world.get_map().get_spawn_points()
    spawn_points_choosen = []

    for points in spawn_points_ori:
      if (points.location.x < x_max) and (points.location.x > x_min) and (points.location.y < y_max) and (points.location.y > y_min):
         spawn_points_choosen.append(points)
    for y in range(-100,100,40):
      points = Transform()
      points.location.x = 203.5
      points.location.y = y
      points.location.z = 2
      points.rotation.pitch = 0
      points.rotation.yaw = -90 
      points.rotation.roll = 0
      spawn_points_choosen.append(points)

    # for y in range(-100,100,10):
    #   points = Transform()
    #   points.location.x = 207
    #   points.location.y = y
    #   points.location.z = 2
    #   points.rotation.pitch = 0
    #   points.rotation.yaw = -90 
    #   points.rotation.roll = 0
    #   spawn_points_choosen.append(points)

    for y in range(-100,100,40):
      points = Transform()
      points.location.x = 211
      points.location.y = y
      points.location.z = 2
      points.rotation.pitch = 0
      points.rotation.yaw = -90 
      points.rotation.roll = 0
      spawn_points_choosen.append(points)

    for y in range(-100,100,10):
      points = Transform()
      points.location.x = 195
      points.location.y = y
      points.location.z = 2
      points.rotation.pitch = 0
      points.rotation.yaw = 90 
      points.rotation.roll = 0
      spawn_points_choosen.append(points)

    for y in range(-100,100,10):
      points = Transform()
      points.location.x = 192
      points.location.y = y
      points.location.z = 2
      points.rotation.pitch = 0
      points.rotation.yaw = 90 
      points.rotation.roll = 0
      spawn_points_choosen.append(points)
    
    blueprints = [x for x in blueprints_ori if int(x.get_attribute('number_of_wheels')) == 4]
    blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
    blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
    blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
    blueprints = [x for x in blueprints if not x.id.endswith('t2')]

    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor
    synchronous_master = True
    # --------------
    # Spawn vehicles
    # --------------
    batch = []
    max_agents = self.number_of_vehicles #70
    #recommended_points = [1, 2, 3, 5, 7, 11, 12, 18, 20, 21, 22, 43, 44, 65, 71, 72, 75, 76, 77, 78, 83, 84, 102, 103, 110, 111, 118, 119, 124, 135, 136, 150, 158, 168, 173, 176, 203, 205, 238, 239, 249, 250, 274, 275]
    #[2,3,13,14,153,154,77,78,51,52,65,66,85,86,199,200,71,72,89,90,93,94,166,168,175,181,116,117] #fewer vehicles
    actor_list = carla_world.get_actors()
    vehicle_list = actor_list.filter("*vehicle*")
    
    
    self.tm.global_percentage_speed_difference(70)
    for vehicle in vehicle_list:
        if has_set[vehicle.id] == 0:
            has_set[vehicle.id] = 1
            self.tm.ignore_vehicles_percentage(vehicle, 30)
        # force lane change
        if vehicle.attributes['role_name'] == "lane_change_vehicle":
            self.tm.force_lane_change(vehicle, random.choice([True, False]))
            self.tm.ignore_vehicles_percentage(vehicle, 100)

    num_agents = len(vehicle_list)
    added_vehicle_num = max_agents - num_agents
    if added_vehicle_num > 2:
        added_vehicle_num = 2

    spawn_that_vehicle = False
    while len(batch) < added_vehicle_num:
       
        transform = random.choice(spawn_points_choosen)
        print("[DEBUG!!] : spawn vehicle at:",transform.location.x,transform.location.y)
        too_closed_to_ego = False
        min_d = 100
        for vehicle in vehicle_list:
            d = vehicle.get_location().distance(transform.location)
            if vehicle.attributes['role_name'] == "ego_vehicle" and d < self.obs_range:
                too_closed_to_ego = True
                break
            if d < min_d:
                min_d = d
            if min_d < 10:
                break
        if min_d < 10 or too_closed_to_ego == True:
            continue
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        blueprint.set_attribute('role_name', 'autopilot')
        batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))    
            
    # print("[CARLA_GYM] : Spawn a vehicle")
    self.client.apply_batch_sync(batch, synchronous_master)

  def removed_too_far_vehicle(self, carla_world):
    global stopped_time

    actor_list = carla_world.get_actors()
    vehicle_list = actor_list.filter("*vehicle*")

    for vehicle in vehicle_list:
        
        if vehicle.attributes['role_name'] == "ego_vehicle":
            continue
        
        if stopped_time[vehicle.id] < -100:
            continue

        v_loc = vehicle.get_location()
        if (v_loc.x > x_max) or (v_loc.x < x_min) or (v_loc.y > y_max) or (v_loc.y < y_min):
            print("[CARLA_GYM] : Delete vehicle move too far",v_loc.x, v_loc.y)
            stopped_time[vehicle.id] = -100000
            vehicle.destroy()
            continue
        
        velocity = vehicle.get_velocity()
        dist_from_origin = vehicle.get_location().distance(circle_center)
        if stopped_time[vehicle.id] >= 0:
            if abs(velocity.x) < 0.01 and abs(velocity.y) < 0.01:
                stopped_time[vehicle.id] = stopped_time[vehicle.id] + 1
            else:
                stopped_time[vehicle.id] = 0

  def removed_stopped_vehicle(self, carla_world, stopped_time_thres = 1000):
     global stopped_time
     actor_list = carla_world.get_actors()
     vehicle_list = actor_list.filter("*vehicle*")
     for vehicle in vehicle_list:
        if stopped_time[vehicle.id] > stopped_time_thres:
            print("[CARLA_GYM] : Delete vehicle stay too long", stopped_time[vehicle.id], velocity.x, velocity.y)
            stopped_time[vehicle.id] = -100000
            vehicle.destroy()

  def adjust_ref_path(self, dao):
    global trajectory_state
    # FIXME(zhcao): trajectory_state indicates if the trajectory is the first (1) or the second one (2).
    ego_x = self.ego.get_transform().location.x
    ego_y = self.ego.get_transform().location.y
    if math.pow((ego_x - second_start.location.x),2) + math.pow((ego_y - second_start.location.y),2) < 100:
        if trajectory_state != 2:              
            self.calculate_route(second_start, second_goal, dao)
            trajectory_state = 2
            print("[CARLA GYM] : Calculated second path")
        return
        
    if math.pow((ego_x - first_start.location.x),2) + math.pow((ego_y - first_start.location.y),2) < 100:
        if trajectory_state != 1:    
            self.calculate_route(first_start, first_goal, dao)
            trajectory_state = 1
            print("[CARLA GYM] : Calculated first path")

    return

  def updated_rendering_result(self, h_reconstruct, reconstruct_sample):
    self.h_reconstruct = h_reconstruct
    self.reconstruct_sample = reconstruct_sample

  def spawn_that_vehicle(self, carla_world, delay = 0.05):
    batch = []

    blueprints_ori = carla_world.get_blueprint_library().filter('vehicle.*')
    
    blueprints = [x for x in blueprints_ori if int(x.get_attribute('number_of_wheels')) == 4]
    blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
    blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
    blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
    blueprints = [x for x in blueprints if not x.id.endswith('t2')]

    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor
    synchronous_master = True
    
    # Spawn a vehicle in front of me 
    blueprint = random.choice(blueprints)
    if blueprint.has_attribute('color'):
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)
    if blueprint.has_attribute('driver_id'):
        driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
        blueprint.set_attribute('driver_id', driver_id)
    blueprint.set_attribute('role_name', 'lane_change_vehicle')
    # blueprint.set_attribute('role_name', 'lane_follow_vehicle')
    temp_vehicle = Transform()
    temp_vehicle.location.x = 206
    temp_vehicle.location.y = 78
    temp_vehicle.location.z = 2
    temp_vehicle.rotation.pitch = 0
    temp_vehicle.rotation.yaw = -90
    temp_vehicle.rotation.roll = 0
    batch.append(SpawnActor(blueprint, temp_vehicle).then(SetAutopilot(FutureActor, True)))  

    temp_vehicle2 = Transform()
    temp_vehicle2.location.x = 203
    temp_vehicle2.location.y = 73
    temp_vehicle2.location.z = 2
    temp_vehicle2.rotation.pitch = 0
    temp_vehicle2.rotation.yaw = -90
    temp_vehicle2.rotation.roll = 0
    batch.append(SpawnActor(blueprint, temp_vehicle2).then(SetAutopilot(FutureActor, True))) 

    self.client.apply_batch_sync(batch, synchronous_master)
 
import os
import glob
import pandas as pd
import commonroad
from commonroad.scenario.state import InitialState
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.geometry.shape import Rectangle
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from utils import save_scenario_at_timestep

import sys
sys.path.insert(0, "/home/kareem/frenet_optimal_trajectory_planner/CVAE/commonroad-reactive-planner")

from commonroad_rp.utility.evaluation import create_full_solution_trajectory
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration
import commonroad_rp
print(commonroad_rp.__file__)
import matplotlib
matplotlib.use('Agg')  # Non-GUI backend (renders to image files)

dir = "/home/kareem/frenet_optimal_trajectory_planner/commonroad_utils/scenarios_w_goals"
config_file = "/home/kareem/frenet_optimal_trajectory_planner/CVAE/config/reactive_planner_config.yaml"
save_dir = "/home/kareem/my-frenet/FrenetOptimalTrajectory/scenarios_modified"

sampled_vars = {
      "scenario": [],
      "t": [],
      "d": [],
      "lon_velocity": [],
}

conditioned_vars = {
      "scenario": [],
      "init_x": [],
      "init_y": [],
      "init_theta": [],
      "init_velocity": [],
      "goal_x": [],
      "goal_y": [],
      # "reach_sets": [],
      # "obstacles": [],
}

for sc in os.listdir(dir):
      if sc.endswith(".xml"):
            img_save_dir = f"/home/kareem/frenet_optimal_trajectory_planner/CVAE/data/scenarios_imgs/{sc[:-4]}"
            # sc_path = os.path.join(dir, sc)
            
            print(f"Planning for {sc}")
            
            config = ReactivePlannerConfiguration.load(config_file, sc)
            config.update()
            
            try:
                  # run route planner
                  route_planner = RoutePlanner(config.scenario, config.planning_problem)
                  route = route_planner.plan_routes().retrieve_first_route()
                  # print(config.planning_problem.initial_state.orientation)
                  # print(config.planning_problem._goal_region.state_list[0].position.center)
                  
                  # Scenario has attribute position as a lanelet in goal (get the center of the lanelet) 
                  if isinstance(config.planning_problem.goal.state_list[0].position, \
                        commonroad.geometry.shape.ShapeGroup):
                        goal_pos = config.planning_problem.goal.state_list[0].position.shapes[0].center
                        
                  # Scenario has attribute position as a rectangle in goal (get the center of the rectangle)
                  elif isinstance(config.planning_problem.goal.state_list[0].position, \
                        commonroad.geometry.shape.Rectangle):
                        goal_pos = config.planning_problem.goal.state_list[0].position.center
                        
                  # get reference path
                  reference_path = route.reference_path
                  
                  planner = ReactivePlanner(config=config)

                  # set reference path for curvilinear coordinate system
                  planner.set_reference_path(route.reference_path)
                  while not planner.goal_reached():

                        planner.set_desired_velocity(current_speed=planner.x_0.velocity)

                        # call plan function
                        optimal, samples = planner.plan()

                        # record planned state and input
                        planner.record_state_and_input(optimal[0].state_list[1])

                        # reset planner state for re-planning
                        planner.reset(initial_state_cart=planner.record_state_list[-1], 
                                    initial_state_curv=(optimal[2][1], optimal[3][1]),
                                    collision_checker=planner.collision_checker, 
                                    coordinate_system=planner.coordinate_system)
                        
                        save_scenario_at_timestep(sc[:-4], planner.record_state_list[-1].time_step)
                        
                        sampled_vars["scenario"].append(sc[:-4])
                        sampled_vars["t"].append(samples[0])
                        sampled_vars["d"].append(samples[1])
                        sampled_vars["lon_velocity"].append(samples[2])
                        
                        conditioned_vars["scenario"].append(sc[:-4])
                        conditioned_vars["init_x"].append(config.planning_problem.initial_state.position[0])
                        conditioned_vars["init_y"].append(config.planning_problem.initial_state.position[1])
                        conditioned_vars["init_theta"].append(config.planning_problem.initial_state.orientation)
                        conditioned_vars["init_velocity"].append(config.planning_problem.initial_state.velocity)
                        conditioned_vars["goal_x"].append(goal_pos[0])
                        conditioned_vars["goal_y"].append(goal_pos[1])

            except Exception as e:
                  print(f"Scenario {sc} failed!")
                  
                  # If the scenario fails, delete all files related to it
                  for file_path in glob.glob(os.path.join(img_save_dir, "*")):
                        if os.path.isfile(file_path):
                              os.remove(file_path)
                              print(f"Deleted: {file_path}")
                  if os.path.exists(img_save_dir) and os.path.isdir(img_save_dir):
                        os.rmdir(img_save_dir)
                        print(f"Deleted directory: {img_save_dir}")
                  continue
            
sampled_vars_df = pd.DataFrame(sampled_vars)
sampled_vars_df.to_csv('data/sampled_vars.csv')
            
conditioned_vars_df = pd.DataFrame(conditioned_vars)
conditioned_vars.to_csv('data/conditioned_vars.csv')
                        
            
            # ego_solution_trajectory = create_full_solution_trajectory(config, planner.record_state_list)
            
            # dynamic_obstacle_initial_state = InitialState(
            #       position=config.planning_problem.initial_state.position,
            #       orientation=config.planning_problem.initial_state.orientation,
            #       velocity=config.planning_problem.initial_state.velocity,
            #       time_step=config.planning_problem.initial_state.time_step,
            #       yaw_rate=0,
            #       slip_angle=0,
            # )
            
            # dynamic_obstacle_shape = Rectangle(width=1.8, length=4.3)
            # dynamic_obstacle_prediction = TrajectoryPrediction(
            #       ego_solution_trajectory, dynamic_obstacle_shape
            # )
            
            # # generate the dynamic obstacle according to the specification
            # dynamic_obstacle_id = config.scenario.generate_object_id()
            # dynamic_obstacle_type = ObstacleType.CAR
            # ego_vehicle = DynamicObstacle(
            #       dynamic_obstacle_id,
            #       dynamic_obstacle_type,
            #       dynamic_obstacle_shape,
            #       dynamic_obstacle_initial_state,
            #       dynamic_obstacle_prediction,
            # )
            
            # config.scenario.add_objects(ego_vehicle)
            
            # fw = CommonRoadFileWriter(
            #       config.scenario,
            #       config.planning_problem_set,
            #       config.scenario.author,
            #       config.scenario.affiliation,
            #       config.scenario.source,
            #       config.scenario.tags
            # )
            
#             fw.write_to_file(os.path.join(save_dir, sc), OverwriteExistingFile.ALWAYS)
            
#             scenario_ego_id['scenario'].append(sc)
#             scenario_ego_id["ego_id"].append(dynamic_obstacle_id)

# sc_ego_id_df = pd.DataFrame(scenario_ego_id)
# sc_ego_id_df.to_csv('scenario_ego_id.csv')
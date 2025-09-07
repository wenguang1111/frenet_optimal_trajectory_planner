import os
# import glob
import numpy as np
import pandas as pd
import commonroad
from commonroad.scenario.state import InitialState
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.geometry.shape import Rectangle
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from cvae.model.utils import save_scenario_imgs

from commonroad_rp.utility.evaluation import create_full_solution_trajectory
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration
import commonroad_rp
print(commonroad_rp.__file__)
import matplotlib
matplotlib.use('Agg')  # Non-GUI backend (renders to image files)
from tqdm import tqdm

dir = "cvae/scenarios_v2"
config_file = "cvae/config/reactive_planner_config.yaml"

# planned = np.array([])
# planned = np.load("cvae/data/data_extended/planned_scenarios.npy")
# print(f"Planned scenarios loaded: {len(set(planned))}")

sampled_vars = {
      "scenario": [],
      "time_step": [],
      "t": [],
      "d": [],
      "lon_velocity": [],
}

conditioned_vars = {
      "scenario": [],
      "time_step": [],
      "x": [],
      "y": [],
      "theta": [],
      "velocity": [],
      "acceleration": [],
      "yaw_rate": [],
      # "goal_x": [],
      # "goal_y": [],
} # maybe add d from commonroad

for sc in tqdm(os.listdir(dir), desc="Planning scenarios", unit="scenario"):
      if sc.endswith(".xml") and sc not in os.listdir("cvae/stubborn"):            
            # if sc in planned or sc in os.listdir("cvae/stubborn"):
            #       print(f"Scenario {sc} already planned, skipping...")
            #       continue
            # else:
            #       print(f"Planning scenario {sc}...") 
            #       planned = np.append(planned, sc)
            #       np.save("cvae/data/data_extended/planned_scenarios.npy", planned)

            # temporary storage for sampled variables
            tmp_sampled = {
                  # "time_step": [],
                  "t": [],
                  "d": [],
                  "lon_velocity": [],
            }
            
            # temporary storage for conditioned variables
            tmp_conditioned = {
                  # "time_step": [],
                  "x": [],
                  "y": [],
                  "theta": [],
                  "velocity": [],
                  "acceleration": [],
                  "yaw_rate": [],
            }
            
            img_save_dir = f"cvae/data/data_extended/scenarios_imgs/{sc[:-4]}"
            # sc_path = os.path.join(dir, sc)
            
            print(f"Planning for {sc}")
            
            config = ReactivePlannerConfiguration.load(config_file, sc)
            config.update()

            try:
                  # run route planner
                  route_planner = RoutePlanner(config.scenario, config.planning_problem)
                  route = route_planner.plan_routes().retrieve_first_route()
                  
                  # Scenario has attribute position as a lanelet in goal (get the center of the lanelet) 
                  # if isinstance(config.planning_problem.goal.state_list[0].position, \
                  #       commonroad.geometry.shape.ShapeGroup):
                  #       goal_pos = config.planning_problem.goal.state_list[0].position.shapes[0].center
                        
                  # # Scenario has attribute position as a rectangle in goal (get the center of the rectangle)
                  # elif isinstance(config.planning_problem.goal.state_list[0].position, \
                  #       commonroad.geometry.shape.Rectangle):
                  #       goal_pos = config.planning_problem.goal.state_list[0].position.center
                  
                  tmp_conditioned["x"].append(config.planning_problem.initial_state.position[0])
                  tmp_conditioned["y"].append(config.planning_problem.initial_state.position[1])
                  tmp_conditioned["theta"].append(config.planning_problem.initial_state.orientation)
                  tmp_conditioned["velocity"].append(config.planning_problem.initial_state.velocity)
                  tmp_conditioned["acceleration"].append(config.planning_problem.initial_state.acceleration)
                  tmp_conditioned["yaw_rate"].append(config.planning_problem.initial_state.yaw_rate)
                        
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

                        samples = np.array(samples)

                        tmp_sampled["t"].append(samples[:, 0].tolist())
                        tmp_sampled["d"].append(samples[:, 1].tolist())
                        tmp_sampled["lon_velocity"].append(samples[:, 2].tolist())
                        
                        # conditioned_vars["scenario"].append(sc[:-4])
                        tmp_conditioned["x"].append(planner.record_state_list[-1].position[0])
                        tmp_conditioned["y"].append(planner.record_state_list[-1].position[1])
                        tmp_conditioned["theta"].append(planner.record_state_list[-1].orientation)
                        tmp_conditioned["velocity"].append(planner.record_state_list[-1].velocity)
                        tmp_conditioned["acceleration"].append(planner.record_state_list[-1].acceleration)
                        tmp_conditioned["yaw_rate"].append(planner.record_state_list[-1].yaw_rate)
                  
                  # save sampled variables and conditiobned variables if scenario is successfully planned
                  if planner.goal_reached():
                        print(f"Scenario {sc} successfully planned!")
                        # print(f"Number of samples: {planner.record_state_list[-1].time_step}")
                        save_scenario_imgs(sc[:-4], planner.record_state_list[-1].time_step)
                        
                        for idx, (t_list, d_list, lon_v_list) in \
                              enumerate(zip(tmp_sampled["t"], tmp_sampled["d"], tmp_sampled["lon_velocity"])):
                                    
                              for t, d, lon_v in zip(t_list, d_list, lon_v_list):
                                    sampled_vars["scenario"].append(sc[:-4])
                                    sampled_vars["time_step"].append(idx)
                                    sampled_vars["t"].append(t)
                                    sampled_vars["d"].append(d)
                                    sampled_vars["lon_velocity"].append(lon_v)
                              # sampled_vars["scenario"].append(sc[:-4])
                              # sampled_vars["t"].append(t)
                              # sampled_vars["d"].append(d)
                              # sampled_vars["lon_velocity"].append(lon_v)
                              
                        for idx, (x, y, theta, velocity, acceleration, yaw_rate) in enumerate(zip(
                              tmp_conditioned["x"][:-1], 
                              tmp_conditioned["y"][:-1], 
                              tmp_conditioned["theta"][:-1], 
                              tmp_conditioned["velocity"][:-1], 
                              tmp_conditioned["acceleration"][:-1], 
                              tmp_conditioned["yaw_rate"][:-1])):
                              
                              conditioned_vars["scenario"].append(sc[:-4])
                              conditioned_vars["time_step"].append(idx)
                              conditioned_vars["x"].append(x)
                              conditioned_vars["y"].append(y)
                              conditioned_vars["theta"].append(theta)
                              conditioned_vars["velocity"].append(velocity)
                              conditioned_vars["acceleration"].append(acceleration)
                              conditioned_vars["yaw_rate"].append(yaw_rate)
                              
                              # conditioned_vars["goal_x"].append(goal_pos[0])
                              # conditioned_vars["goal_y"].append(goal_pos[1])
                              
            except Exception as e:
                  print(f"Scenario {sc} failed!")
                  # print(e)
                  continue
            
sampled_vars_df = pd.DataFrame(sampled_vars)
sampled_vars_df.to_csv('cvae/data/data_extended/sampled_vars.csv')
            
conditioned_vars_df = pd.DataFrame(conditioned_vars)
conditioned_vars_df.to_csv('cvae/data/data_extended/conditioned_vars.csv')
                        
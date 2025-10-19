import os
# import glob
import traceback
import numpy as np
import pandas as pd
from commonroad.scenario.state import InitialState
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.geometry.shape import Rectangle
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from cvae.utils.utils import save_scenario_imgs

from commonroad_rp.utility.evaluation import create_full_solution_trajectory
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration
import commonroad_rp
print(commonroad_rp.__file__)
import matplotlib
matplotlib.use('Agg')  # Non-GUI backend (renders to image files)
from tqdm import tqdm

from commonroad_rp.utility.logger import initialize_logger

# initialize and get logger
logger = initialize_logger(ReactivePlannerConfiguration())

sc_dir = "cvae/all_scenarios"
save_scenarios_dir = "cvae/all_scenarios/planned_scenarios"
data_save_dir = "cvae/data/data_v2"
img_save_dir = "cvae/data/data_v2/scenarios_imgs"
os.makedirs(save_scenarios_dir, exist_ok=True)
os.makedirs(img_save_dir, exist_ok=True)
config_file = "cvae/config/reactive_planner_config_rp.yaml"

ego_id = {
    'scenario': [],
    'ego_id': []
}

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
} # maybe add d from commonroad

for sc in tqdm(os.listdir(sc_dir), desc="Planning scenarios", unit="scenario"):
    if sc.endswith(".xml"):

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
        
        logger.info(f"Planning for {sc}")
        
        config = ReactivePlannerConfiguration.load(config_file, sc)
        config.update()

        try:
            # run route planner
            route_planner = RoutePlanner(config.scenario, config.planning_problem)
            route = route_planner.plan_routes().retrieve_first_route()
            
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
                logger.info(f"Scenario {sc} successfully planned!")
                    
                for idx, (t_list, d_list, lon_v_list) in \
                    enumerate(zip(tmp_sampled["t"], tmp_sampled["d"], tmp_sampled["lon_velocity"])):
                            
                    for t, d, lon_v in zip(t_list, d_list, lon_v_list):
                        sampled_vars["scenario"].append(sc[:-4])
                        sampled_vars["time_step"].append(idx)
                        sampled_vars["t"].append(t)
                        sampled_vars["d"].append(d)
                        sampled_vars["lon_velocity"].append(lon_v)
                            
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
                        
                ego_solution_trajectory = create_full_solution_trajectory(config, planner.record_state_list)
    
                dynamic_obstacle_initial_state = InitialState(
                    position=config.planning_problem.initial_state.position,
                    orientation=config.planning_problem.initial_state.orientation,
                    velocity=config.planning_problem.initial_state.velocity,
                    time_step=config.planning_problem.initial_state.time_step,
                    yaw_rate=0,
                    slip_angle=0,
                )
                
                dynamic_obstacle_shape = Rectangle(width=1.8, length=4.3)
                dynamic_obstacle_prediction = TrajectoryPrediction(
                    ego_solution_trajectory, dynamic_obstacle_shape
                )
    
                # generate the dynamic obstacle according to the specification
                dynamic_obstacle_id = config.scenario.generate_object_id()
                dynamic_obstacle_type = ObstacleType.CAR
                ego_vehicle = DynamicObstacle(
                    dynamic_obstacle_id,
                    dynamic_obstacle_type,
                    dynamic_obstacle_shape,
                    dynamic_obstacle_initial_state,
                    dynamic_obstacle_prediction,
                )
                
                config.scenario.add_objects(ego_vehicle)
                
                fw = CommonRoadFileWriter(
                    config.scenario,
                    config.planning_problem_set,
                    config.scenario.author,
                    config.scenario.affiliation,
                    config.scenario.source,
                    config.scenario.tags
                )
                
                fw.write_to_file(os.path.join(save_scenarios_dir, sc), OverwriteExistingFile.ALWAYS)
                
                save_scenario_imgs(
                    sc_path=save_scenarios_dir,
                    save_dir=img_save_dir,
                    scenario_name=sc[:-4],
                    ego_id=dynamic_obstacle_id,
                    no_time_step=planner.record_state_list[-1].time_step
                )
                
                ego_id['scenario'].append(sc)
                ego_id["ego_id"].append(dynamic_obstacle_id)
                              
        except Exception as e:
            logger.info(f"Scenario {sc} failed!")
            # print(traceback.format_exc())
            continue
            
sampled_vars_df = pd.DataFrame(sampled_vars)
conditioned_vars_df = pd.DataFrame(conditioned_vars)
ego_id_df = pd.DataFrame(ego_id)

# repeat the conditioned variables for each sample
data_merged = pd.merge(sampled_vars_df, conditioned_vars_df, on=["scenario", "time_step"], how="inner")
cols_to_drop = [data_merged.columns[i] for i in range(2, 5)]
conditioned_vars_repeated = data_merged.drop(columns=cols_to_drop)

# save dataframes to parquet files
sampled_vars_df.to_parquet(data_save_dir + '/sampled_vars.parquet', index=False)
conditioned_vars_repeated.to_parquet(data_save_dir + '/conditioned_vars.parquet', index=False)
ego_id_df.to_parquet(data_save_dir + '/ego_ids.parquet', index=False)

print("Data preparation completed!")
print("Samples shape: ", sampled_vars_df.shape)
print("Conditions shape:", conditioned_vars_repeated.shape)


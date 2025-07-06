import os
from commonroad.common.file_reader import CommonRoadFileReader
import matplotlib.pyplot as plt
# import numpy as np
# from commonroad.planning.planning_problem import PlanningProblemSet
# from commonroad.scenario.scenario import Scenario
from commonroad.visualization.mp_renderer import MPRenderer


def save_scenario_at_timestep(
    scenario_name: str,
    time_step: int, 
) -> None:
      """
      save an image of the scenarios at a specific timestep.
      
      Args:
            Scenario name.
            Time step.
      """
      save_dir = "/home/kareem/frenet_optimal_trajectory_planner/CVAE/data/scenarios_imgs"
      sc_path = "/home/kareem/frenet_optimal_trajectory_planner/CVAE/scenarios"
      scenario, pp = CommonRoadFileReader(os.path.join(sc_path, scenario_name) + ".xml").open()
      
      renderer = MPRenderer()
      renderer.draw_params.axis_visible = False
      renderer.draw_params.time_begin = time_step
      renderer.draw_params.dynamic_obstacle.draw_shape = True
      renderer.draw_params.dynamic_obstacle.draw_icon = True
      scenario.draw(renderer)
      pp.draw(renderer)
      
      plt.gca().set_aspect("equal")
      renderer.render()
      os.makedirs(save_dir + "/" + scenario_name, exist_ok=True)
      plt.savefig(save_dir + f"/{scenario_name}/time_step_{time_step}.png")
      
      
def remove_entries_for_scenario(data_dict, scenario_name):
    # Create a mask of indices to keep (where scenario is NOT the one to remove)
    scenario_list = data_dict["scenario"]
    indices_to_keep = [i for i, s in enumerate(scenario_list) if s != scenario_name]

    # Apply the same filtering to every key
    for key in data_dict:
        data_dict[key] = [data_dict[key][i] for i in indices_to_keep]


    
    
if __name__ == "__main__":
      save_scenario_at_timestep("DEU_Flensburg-10_1_T-1", 5)
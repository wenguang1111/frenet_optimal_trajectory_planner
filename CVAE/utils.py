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
            Planning problem object.
      """
      save_dir = "/home/kareem/frenet_optimal_trajectory_planner/CVAE/data/scenarios_imgs"
      sc_path = "/home/kareem/frenet_optimal_trajectory_planner/CVAE/scenarios"
      scenario, _ = CommonRoadFileReader(os.path.join(sc_path, scenario_name) + ".xml").open()
      
      renderer = MPRenderer()
      renderer.draw_params.axis_visible = False
      renderer.draw_params.time_begin = time_step
      renderer.draw_params.dynamic_obstacle.draw_shape = True
      renderer.draw_params.dynamic_obstacle.draw_icon = True
      scenario.draw(renderer)
      
      plt.gca().set_aspect("equal")
      renderer.render()
      os.makedirs(save_dir + "/" + scenario_name, exist_ok=True)
      plt.savefig(save_dir + f"/{scenario_name}/time_step_{time_step}.png")
    
    
save_scenario_at_timestep("DEU_Flensburg-8_1_T-1", 0)
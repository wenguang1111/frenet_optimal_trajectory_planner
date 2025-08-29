import os
from commonroad.common.file_reader import CommonRoadFileReader
import matplotlib.pyplot as plt
# import numpy as np
# from commonroad.planning.planning_problem import PlanningProblemSet
# from commonroad.scenario.scenario import Scenario
from commonroad.visualization.mp_renderer import MPRenderer


def save_scenario_imgs(
    scenario_name: str,
    no_time_step: int, 
) -> None:
      """
      save an image of the scenarios at a specific timestep.
      
      Args:
            Scenario name.
            Time step.
      """
      save_dir = "cvae/data/data_extended/scenarios_imgs"
      sc_path = "cvae/scenarios_v2"
      os.makedirs(save_dir + "/" + scenario_name, exist_ok=True)
      
      scenario, pp = CommonRoadFileReader(os.path.join(sc_path, scenario_name) + ".xml").open()
      
      for ts in range(0, no_time_step + 1):
            renderer = MPRenderer()
            renderer.draw_params.axis_visible = False
            renderer.draw_params.time_begin = ts
            renderer.draw_params.dynamic_obstacle.draw_shape = True
            renderer.draw_params.dynamic_obstacle.draw_icon = True
            scenario.draw(renderer)
            pp.draw(renderer)
            
            plt.gca().set_aspect("equal")
            renderer.render()
            
            plt.savefig(save_dir + f"/{scenario_name}/time_step_{ts}.png")
            
    
if __name__ == "__main__":
      save_scenario_imgs("DEU_Flensburg-10_1_T-1", 5)
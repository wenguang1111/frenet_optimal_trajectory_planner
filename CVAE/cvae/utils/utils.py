import os
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.draw_params import DynamicObstacleParams


def save_scenario_imgs(
    sc_path: str,
    save_dir: str,
    scenario_name: str,
    ego_id: int,
    no_time_step: int,
) -> None:
      """
      save an image of the scenarios at a specific timestep.
      
      Args:
            Scenarios path.
            Img save directory.
            Scenario name.
            Ego vehicle id.
            Time step.
      """

      os.makedirs(save_dir + "/" + scenario_name, exist_ok=True)
      
      scenario, pp = CommonRoadFileReader(os.path.join(sc_path, scenario_name) + ".xml").open()
      
      # retrieve ego vehicle by id and set parameters for visualization
      ego_vehicle = scenario.obstacle_by_id(ego_id)
      ego_params = DynamicObstacleParams()
      ego_params.vehicle_shape.occupancy.shape.facecolor = "#ff0000"
      ego_params.draw_icon = True
      
      # loop through all time steps and save images
      for ts in range(0, no_time_step + 1):
            renderer = MPRenderer()
            # focus on ego vehicle
            renderer.focus_obstacle_id = ego_id
            renderer.draw_params.axis_visible = False
            renderer.draw_params.time_begin = ts
            renderer.draw_params.dynamic_obstacle.draw_shape = True
            renderer.draw_params.dynamic_obstacle.draw_icon = True
            
            ego_params.time_begin = ts
            
            scenario.draw(renderer)
            # pp.draw(renderer)
            
            ego_vehicle.draw(renderer, draw_params=ego_params)
            
            plt.gca().set_aspect("equal")
            renderer.render()
            
            # save img without padding
            plt.savefig(
                save_dir + f"/{scenario_name}/time_step_{ts}.png",
                bbox_inches='tight',
                pad_inches=0,
                dpi=300,
            )
            
    
if __name__ == "__main__":
      save_scenario_imgs("DEU_Flensburg-10_1_T-1", 5)
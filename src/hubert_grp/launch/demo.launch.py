from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("hubert", package_name="hubert_grp").planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        ).to_moveit_configs())
    return generate_demo_launch(moveit_config)


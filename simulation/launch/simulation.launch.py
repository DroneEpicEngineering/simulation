# from pathlib import Path
from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    simulation_share = get_package_share_path("simulation")

    run_gazebo_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", (simulation_share / "worlds" / "basic.sdf").as_posix()],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(run_gazebo_sim)

    return ld

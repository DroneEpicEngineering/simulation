import os
from pathlib import Path
from ament_index_python import get_package_share_path, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import SetParameter


def generate_launch_description():
    simulation_share = get_package_share_path("simulation")
    target_prefix = Path(get_package_prefix("target"))
    px4_path = Path(os.environ.get("PX4_PATH"))
    px4_bin = px4_path / "build" / "px4_sitl_default" / "bin" / "px4"

    resources = [
        (px4_path / "Tools" / "simulation" / "gz" / "models"),
        (simulation_share / "models"),
    ]

    plugins = [(target_prefix / "lib" / "target")]

    set_resources = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", ":".join(resource.as_posix() for resource in resources)
    )

    set_plugins = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH", ":".join(plugin.as_posix() for plugin in plugins)
    )

    set_sim_time = SetParameter("use_sim_time", value=True)

    run_gazebo_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", (simulation_share / "worlds" / "basic.sdf").as_posix()],
        output="screen",
    )

    run_autopilot = ExecuteProcess(
        cmd=[px4_bin.as_posix()],
        additional_env={
            "PX4_SYS_AUTOSTART": "4001",
            "PX4_SIM_MODEL": "x500_vision",
            "PX4_GZ_MODEL_POSE": "0 0 0.17696 0 0 0",
        },
        output="screen",
    )

    run_qgroundcontrol = ExecuteProcess(
        cmd=[(Path.home() / "QGroundControl-x86_64.AppImage").as_posix()],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(set_resources)
    ld.add_action(set_plugins)
    ld.add_action(set_sim_time)
    ld.add_action(run_gazebo_sim)
    ld.add_action(run_autopilot)
    ld.add_action(run_qgroundcontrol)

    return ld

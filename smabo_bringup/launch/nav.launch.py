"""The navigation stack: Nav2 servers + nav_manager.

Started and stopped by mode_manager (smabo-web's Navigation tab), which also
switches robot_state_publisher to the mobile-base URDF first. Can be run by
hand on top of bringup.launch.py too.

nav_manager runs either SLAM (mapping; Cartographer by default) or
map_server+AMCL (navigation) and switches between them on request from smabo-web.

Arguments:
  slam      (false) start in mapping mode (SLAM) even when a map is given.
  slam_backend  (cartographer) SLAM for mapping mode: cartographer | slam_toolbox.
            slam_toolbox does not correct turning in place.
  map       map to start AMCL on: a yaml path or the name of a saved map. Empty
            (default): start in mapping mode (AMCL on an empty map cannot localize).
  maps_dir  where maps are saved / listed (default: $SMABO_MAPS_DIR, else ~/smabo-maps).
  arm / hand / breadboard / mount  accepted (mode_manager passes the model) but unused.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    navigation = get_package_share_directory("smabo_navigation")
    nav2_params = os.path.join(navigation, "config", "nav2_params.yaml")
    empty_map = os.path.join(navigation, "maps", "empty_map.yaml")

    def map_or_default():
        # an empty map:= (mode_manager passes the bringup value through) means the default
        m = LaunchConfiguration("map")
        return PythonExpression(["'", m, "' or '", empty_map, "'"])

    return LaunchDescription([
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("maps_dir", default_value=""),
        # SLAM for mapping mode: cartographer (tracks turning in place) or slam_toolbox
        DeclareLaunchArgument("slam_backend", default_value="cartographer"),
        *[DeclareLaunchArgument(k, default_value="") for k in ("arm", "hand", "breadboard", "mount")],

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("nav2_bringup"), "launch",
                             "navigation_launch.py")
            ),
            launch_arguments={"params_file": nav2_params, "autostart": "true"}.items(),
        ),
        Node(
            package="smabo_brain_ros",
            executable="nav_manager",
            name="nav_manager",
            output="screen",
            parameters=[{
                "initial_mode": PythonExpression(
                    ["'mapping' if '", LaunchConfiguration("slam"), "' == 'true' or not '",
                     LaunchConfiguration("map"), "' else 'navigation'"]),
                "initial_map": map_or_default(),
                "maps_dir": LaunchConfiguration("maps_dir"),
                "nav2_params": nav2_params,
                "slam_backend": LaunchConfiguration("slam_backend"),
                "slam_params": os.path.join(navigation, "config", "slam_toolbox.yaml"),
            }],
        ),
    ])

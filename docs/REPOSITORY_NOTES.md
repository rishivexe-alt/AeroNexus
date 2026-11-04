# Repository integration notes

## Included project material

This repository combines the original project documentation and Gazebo media
with the separately supplied Python, Bash, XML and YAML files.

## Important external workspace dependencies

`launch_as2_5drone.bash` is an integration launcher, not a standalone installer.
It expects the surrounding AeroStack2 project workspace to provide resources
such as:

- `utils/get_drones.py`
- `utils/stop_tmux_sessions.bash`
- `utils/tmuxinator_to_genome.py` when using GNOME mode
- `tmuxinator/aerostack2.yaml`
- Any additional launch/configuration resources required by the installed
  AeroStack2 project.

The five-drone repository supplies `config/world_swarm_5drone.yaml` and
`config/fastdds_udp_only.xml`; it does not bundle the entire AeroStack2 project.

## Control variants

Run one control variant at a time against the simulated fleet. The programs
publish to overlapping vehicle command interfaces and are not designed to be
run concurrently against the same drones.

## Validation performed during packaging

- Python syntax compilation (`py_compile`) for the four Python programs.
- Bash syntax validation (`bash -n`) for the four Bash scripts.
- XML parsing for the Fast DDS profile.
- YAML parsing where a YAML parser is available.

These are static checks only. A full runtime test requires the target Ubuntu,
ROS 2 Humble, AeroStack2 and Gazebo environment.

## Shutdown caution

Review `stop.bash` before execution. It invokes a helper from the surrounding
workspace and terminates Gazebo-related processes. Land simulated drones first.

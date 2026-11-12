# Repository integration notes

## Included project material

This repository combines the original project documentation and Gazebo media with the Python, Bash, XML and YAML files used in the five-drone project.

| Group | Files |
|---|---|
| Fixed-leader system | `leader_follower_command_demo.py`, `direct_formation.py`, `leader_follower_dds.py`, `leader_tasks.py`, `run_leader_follower.sh`, `run_leader_tasks.sh` |
| Dynamic leader election | `election.py`, `dynamic_swarm.py`, `swarm_cli.py`, `test_election.py`, `run_dynamic_swarm.sh` |
| Simulation | `launch_as2_5drone.bash`, `stop.bash`, `config/world_swarm_5drone.yaml`, `config/fastdds_udp_only.xml` |
| Documentation | `docs/Technical_Documentation.md`, `docs/Technical_Report.pdf`, `docs/Dynamic_Swarm_Technical_Documentation.pdf` |

## Important external workspace dependencies

`launch_as2_5drone.bash` is an integration launcher, not a standalone installer. It expects the surrounding AeroStack2 project workspace to provide:

- `utils/get_drones.py`
- `utils/stop_tmux_sessions.bash`
- `utils/tmuxinator_to_genome.py` (GNOME mode only)
- `tmuxinator/aerostack2.yaml`
- any additional launch resources required by the installed AeroStack2 project

The launcher references `config/world.yaml` for its non-swarm mode. That file is not included because this repository packages the five-drone configuration only.

## Control variants

Run one control variant at a time against the simulated fleet. The programs publish to overlapping vehicle command interfaces and are not designed to run concurrently against the same drones.

## Validation

Automated in CI (`.github/workflows/ci.yml`):

- Python syntax compilation for all programs
- Bash syntax validation for all scripts
- XML and YAML parsing of the configuration files
- Offline election test suite (`python3 test_election.py`)

These checks do not replace a full runtime test, which requires Ubuntu 22.04, ROS 2 Humble, AeroStack2 and Gazebo.

## Shutdown caution

Review `stop.bash` before execution. It invokes a helper from the surrounding workspace and terminates Gazebo-related processes. Land simulated drones first.

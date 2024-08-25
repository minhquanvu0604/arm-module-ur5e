# Simple UR5 Controller
Used for setting up the UR5e manipulator using Universal Robot ROS Driver and MoveIt
- [Setup guide](wiki\setup.md#dependency)
- [Docker](wiki\setup.md#docker)
- [Resources](wiki\resource.md)

## Codebase Structure
- `cpp`: codebase in cpp (Python code maybe be ported to cpp for performance in the end). Not yet developed, only has scratch files 
- `python`: code base in Python, using MoveIt Python API
    - `src`: libraries
        - `collision_manager.py`, `gripper.py`: extra settings for controller, not used for now
        - `UR5e.py`: robot main planner and controller, using MoveIt
        - `utility.py`
    - `scripts.py`: executables to run ROS nodes 
        - `run_robot_controller.py`: ROS node to coordinate the mission of the robot, operate on an instance of the UR5e controller. For testing purposes, it can either:
            - Directly read a json file of poses and controll the robot to reach those goals
            - Receive goals one by one through service call, complete them sequentially
        - `move_robot_client.py`: ROS client to make service call that sends pose goals to the controller
- `launch`: ROS launch files to launch a custom robot (UR5e with added gripper, base, camera, etc). Not yet developed, only has scratch files 
    - Check out the [launch instructions](wiki\launching.md)
- `srv`: set up ROS service
- `urdf`: custom URDF to add camera, based, etc. Not yet developed, only has scratch files 
- `docker`: Docker setup the package


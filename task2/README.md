### Task 2 - Usage instructions

To run amcl_simulation, gmapping_simulation, rins_world and face_localizer at the same time:

`roslaunch task2 combined.launch`

To run map_goals:

`rosrun task2 task2_map_goals`

To enable audio signals (allow robot to greet found faces):

`rosrun sound_play soundplay_node.py`

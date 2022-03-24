### Homework 4 - Usage instructions

To run amcl_simulation, gmapping_simulation, rins_world and face_localizer_dnn at the same time:

`roslaunch task1 combined.launch`

To run map_goals:

`rosrun task1 task1_map_goals`

To enable audio signals (allow robot to greet found faces):

`rosrun sound_play soundplay_node.py`

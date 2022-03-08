# Homework 1

First homework for the subject Development of Intelligent Systems on FRI.

## Key takeaways

### Displaying nodes
We can display nodes and topics visually with `rqt_graph`.

```bash
rosrun rqt_graph rqt_graph
```

### Information about topics
For more information about topics, refer to `rostopic`.

```bash
rostopic -h
```

### Message types
For two nodes to exchange messages they need to use the same message type.

View message types as follows:
```
$ rostopic type /turtle1/command_velocity
turtlesim/Velocity
```

To view the detailed message type structure, use the following:
```
$ rosmsg show turtlesim/Velocity
float32 linear
float32 angular
```

To publish a message from the commandline, use:
```
rostopic pub [topic] [msg_type] [args]
```

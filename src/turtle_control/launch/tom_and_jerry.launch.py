import launch
import launch_ros.actions
from launch.actions import ExecuteProcess
from launch.actions import LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.actions import EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():

    ld = launch.LaunchDescription()

    #Action to start turtlesim node
    turtlesim_node_action = launch_ros.actions.Node(package = 'turtlesim', executable = 'turtlesim_node', name = 'turtlesim')
    ld.add_action(turtlesim_node_action)

    #Action to kill the default turtle
    kill_turtle1_action = ExecuteProcess(cmd=['ros2', 'service', 'call', '/kill', 'turtlesim/srv/Kill', '"{name: turtle1}"'], shell=True)

    #Action to spawn Jerry
    spawn_jerry_action = ExecuteProcess(cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', '"{x: 5.5, y: 5.5, theta: 0, name: jerry}"'], shell=True)

    #Action to spawn Tom 
    spawn_tom_action = ExecuteProcess(cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', '"{x: 1.0, y: 1.0, theta: 0, name: tom}"'], shell=True)
    
    #Action to set off the pen of Jerry
    set_jerry_pen_action = ExecuteProcess(cmd=['ros2', 'service', 'call', '/jerry/set_pen', 'turtlesim/srv/SetPen', """ "{'r': 0, 'g': 0, 'b': 0, 'width': 0, 'off': 1}" """], shell=True)
    
    #Action to run the controller node later
    controller_node_action = launch_ros.actions.Node(package='turtle_control', executable='turtle_tom', name='turtle_chaser')

    setup_sequence_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=turtlesim_node_action,
            on_start=[
                LogInfo(msg='Turtlesim started, configuring simulation...'),

                # Wait 1 second before starting the sequence
                TimerAction(
                    period=1.0,
                    actions=[
                        LogInfo(msg='Executing setup step 1: Kill turtle1...'),
                        kill_turtle1_action, # Run the kill action

                        # AFTER kill finishes, wait 1s then run spawn tom
                        TimerAction(
                            period=1.0,
                            actions=[
                                LogInfo(msg='Executing setup step 2: Spawn Tom...'),
                                spawn_tom_action, # Run spawn tom

                                # AFTER spawn tom, wait 1s then run spawn jerry
                                TimerAction(
                                    period=1.0,
                                    actions=[
                                        LogInfo(msg='Executing setup step 3: Spawn Jerry...'),
                                        spawn_jerry_action, # Run spawn jerry

                                        # AFTER spawn jerry, wait 1s then set pen
                                        TimerAction(
                                            period=1.0,
                                            actions=[
                                                LogInfo(msg='Executing setup step 4: Set Jerry Pen...'),
                                                set_jerry_pen_action, # Run set pen

                                                # AFTER set pen, wait 1.0 second then start chasing
                                                TimerAction(
                                                    period=1.0,
                                                    actions=[
                                                        LogInfo(msg='Simulation configured, setup complete')
                                                    ]
                                                )
                                            ]
                                        )
                                    ]
                                )
                            ]
                        )
                    ]
                )
            ]
        )
    )

    ld.add_action(setup_sequence_handler)

    start_controller_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=turtlesim_node_action,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[
                        LogInfo(msg='Starting controller node'),
                        controller_node_action
                    ]
                )
            ]
        )
    )
    ld.add_action(start_controller_handler)

    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=turtlesim_node_action,
            on_exit=[
                LogInfo(msg='Turtlesim node exited, shutting down launch'),
                EmitEvent(event=Shutdown())
            ]
        )
    )
    ld.add_action(shutdown_handler)
    return ld
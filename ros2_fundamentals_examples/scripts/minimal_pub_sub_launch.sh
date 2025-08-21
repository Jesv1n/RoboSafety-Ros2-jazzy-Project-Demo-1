#!/bin/bash
"""
  This is a wrapper to run both the publisher and subscriber nodes.
"""
cleanup(){
    echo "Restarting ROS 2 daemon to cleanup before shutting down all processes...."
    ros2 daemon stop
    sleep 1
    ros2 daemon start
    echo "Terminating all ROS 2 related processes..."
    kill 0
    exit
}

trap 'cleanup' SIGINT

# Launch publisher node in the background
ros2 run ros2_fundamentals_examples py_minimal_publisher &

# Give it a moment to start
sleep 2

# Launch subscriber node in the background
ros2 run ros2_fundamentals_examples py_minimal_subscriber &

# Wait for both processes to keep the script alive
wait

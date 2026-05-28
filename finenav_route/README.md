ros2 action send_goal /compute_path finenav_msgs/action/ComputePath "{
  graph_name: 'rectangle_centered',
  start_node_id: 0,
  goal_node_id: 5
}"
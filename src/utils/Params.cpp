/**
 * @file Param.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Param class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "utils/Params.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods ----------------------------- */

Params::Params(rclcpp::Node::SharedPtr const nh) {
  std::string ns = nh->get_namespace();
  // Main
  main.package_path = ament_index_cpp::get_package_share_directory("urinay");

  nh->declare_parameter<std::string>(ns + "/input_cones_topic", "");
  main.input_cones_topic = nh->get_parameter(ns + "/input_cones_topic").get_value<std::string>();

  nh->declare_parameter<std::string>(ns + "/input_pose_topic", "");
  main.input_pose_topic = nh->get_parameter(ns + "/input_pose_topic").get_value<std::string>();

  nh->declare_parameter<std::string>(ns + "/output_full_center_topic", "");
  main.output_full_center_topic = nh->get_parameter(ns + "/output_full_center_topic").get_value<std::string>();

  nh->declare_parameter<std::string>(ns + "/output_full_left_topic", "");
  main.output_full_left_topic = nh->get_parameter(ns + "/output_full_left_topic").get_value<std::string>();

  nh->declare_parameter<std::string>(ns + "/output_full_right_topic", "");
  main.output_full_right_topic = nh->get_parameter(ns + "/output_full_right_topic").get_value<std::string>();

  nh->declare_parameter<std::string>(ns + "/output_partial_center_topic", "");
  main.output_partial_center_topic = nh->get_parameter(ns + "/output_partial_center_topic").get_value<std::string>();

  nh->declare_parameter<std::string>(ns + "/output_partial_left_topic", "");
  main.output_partial_left_topic = nh->get_parameter(ns + "/output_partial_left_topic").get_value<std::string>();

  nh->declare_parameter<std::string>(ns + "/output_partial_right_topic", "");
  main.output_partial_right_topic = nh->get_parameter(ns + "/output_partial_right_topic").get_value<std::string>();

  nh->declare_parameter<bool>(ns + "/shutdown_on_loop_closure", true);
  main.shutdown_on_loop_closure = nh->get_parameter(ns + "/shutdown_on_loop_closure").get_value<bool>();

  nh->declare_parameter<float>(ns + "/min_cone_confidence");
  main.min_cone_confidence = nh->get_parameter(ns + "/min_cone_confidence").get_value<float>();

  // WayComputer
  nh->declare_parameter<double>(ns + "/max_triangle_edge_len", 9.0);
  wayComputer.max_triangle_edge_len = nh->get_parameter(ns + "/max_triangle_edge_len").get_value<double>();

  nh->declare_parameter<double>(ns + "/min_triangle_angle", 0.25);
  wayComputer.min_triangle_angle = nh->get_parameter(ns + "/min_triangle_angle").get_value<double>();

  nh->declare_parameter<double>(ns + "/max_dist_circum_midPoint", 1.0);
  wayComputer.max_dist_circum_midPoint = nh->get_parameter(ns + "/max_dist_circum_midPoint").get_value<double>();

  nh->declare_parameter<int>(ns + "/failsafe_max_way_horizon_size", 6);
  wayComputer.failsafe_max_way_horizon_size = nh->get_parameter(ns + "/failsafe_max_way_horizon_size").get_value<int>();

  nh->declare_parameter<bool>(ns + "/general_failsafe", true);
  wayComputer.general_failsafe = nh->get_parameter(ns + "/general_failsafe").get_value<bool>();

  nh->declare_parameter<double>(ns + "/general_failsafe_safetyFactor", 1.4);
  wayComputer.general_failsafe_safetyFactor = nh->get_parameter(ns + "/general_failsafe_safetyFactor").get_value<double>();

  // WayComputer::Search
  nh->declare_parameter<int>(ns + "/max_way_horizon_size", 0);
  wayComputer.search.max_way_horizon_size = nh->get_parameter(ns + "/max_way_horizon_size").get_value<int>();

  nh->declare_parameter<int>(ns + "/max_search_tree_height", 5);
  wayComputer.search.max_search_tree_height = nh->get_parameter(ns + "/max_search_tree_height").get_value<int>();

  nh->declare_parameter<double>(ns + "/search_radius", 10.0);
  wayComputer.search.search_radius = nh->get_parameter(ns + "/search_radius").get_value<double>();

  nh->declare_parameter<double>(ns + "/max_angle_diff", 0.6);
  wayComputer.search.max_angle_diff = nh->get_parameter(ns + "/max_angle_diff").get_value<double>();

  nh->declare_parameter<double>(ns + "/edge_len_diff_factor", 0.5);
  wayComputer.search.edge_len_diff_factor = nh->get_parameter(ns + "/edge_len_diff_factor").get_value<double>();

  nh->declare_parameter<int>(ns + "/max_search_options", 2);
  wayComputer.search.max_search_options = nh->get_parameter(ns + "/max_search_options").get_value<int>();

  nh->declare_parameter<double>(ns + "/max_next_heuristic", 3.0);
  wayComputer.search.max_next_heuristic = nh->get_parameter(ns + "/max_next_heuristic").get_value<double>();

  nh->declare_parameter<float>(ns + "/heur_dist_ponderation", 0.6);
  wayComputer.search.heur_dist_ponderation = nh->get_parameter(ns + "/heur_dist_ponderation").get_value<float>();

  nh->declare_parameter<bool>(ns + "/allow_intersection", false);
  wayComputer.search.allow_intersection = nh->get_parameter(ns + "/allow_intersection").get_value<bool>();

  nh->declare_parameter<float>(ns + "/max_treeSearch_time", 0.05);
  wayComputer.search.max_treeSearch_time = nh->get_parameter(ns + "/max_treeSearch_time").get_value<float>();

  // WayComputer::Way
  nh->declare_parameter<double>(ns + "/max_dist_loop_closure", 1.0);
  wayComputer.way.max_dist_loop_closure = nh->get_parameter(ns + "/max_dist_loop_closure").get_value<double>();

  nh->declare_parameter<double>(ns + "/max_angle_diff_loop_closure", 0.6);
  wayComputer.way.max_angle_diff_loop_closure = nh->get_parameter(ns + "/max_angle_diff_loop_closure").get_value<double>();

  nh->declare_parameter<int>(ns + "/vital_num_midpoints", 5);
  wayComputer.way.vital_num_midpoints = nh->get_parameter(ns + "/vital_num_midpoints").get_value<int>();

  // Visualization
  nh->declare_parameter<bool>(ns + "/publish_markers", false);
  visualization.publish_markers = nh->get_parameter(ns + "/publish_markers").get_value<bool>();

  nh->declare_parameter<std::string>(ns + "/marker_topics/triangulation", "/AS/P/urinay/markers/triangulation");
  visualization.triangulation_topic = nh->get_parameter(ns + "/marker_topics/triangulation").get_value<std::string>();

  nh->declare_parameter<std::string>(ns + "/marker_topics/midpoints", "/AS/P/urinay/markers/midpoints");
  visualization.midpoints_topic = nh->get_parameter(ns + "/marker_topics/midpoints").get_value<std::string>();

  nh->declare_parameter<std::string>(ns + "/marker_topics/way", "/AS/P/urinay/markers/way");
  visualization.way_topic = nh->get_parameter(ns + "/marker_topics/way").get_value<std::string>();
}
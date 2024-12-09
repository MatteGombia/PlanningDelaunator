/**
 * @file main.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Main file of Urinay, creates all modules, subcribers and publishers.
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include <mmr_base/msg/marker_array.hpp>

// #include <as_msgs/ConeArray.h>
// #include <as_msgs/PathLimits.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/stat.h>

#include <iostream>

#include "modules/DelaunayTri.hpp"
#include "modules/Visualization.hpp"
#include "modules/WayComputer.hpp"
#include "utils/Time.hpp"

WayComputer *wayComputer;
Params *params;

rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr pubPartialCenterLine;
rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr pubPartialBorderLeft;
rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr pubPartialBorderRight;

rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr pubFullCenterLine;
rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr pubFullBorderLeft;
rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr pubFullBorderRight;

// This is the map callback
void callback_ccat(const mmr_base::msg::MarkerArray &data)
{
  if (not wayComputer->isLocalTfValid())
  {
    RCLCPP_INFO(rclcpp::get_logger(""), "[urinay] CarState not being received.");
    return;
  }
  if (data.markers.empty())
  {
    RCLCPP_INFO(rclcpp::get_logger(""), "[urinay] reading empty set of cones.");
    return;
  }

  Time::tick("computation"); // Start measuring time

  // Convert to Node vector
  std::vector<Node> nodes;
  nodes.reserve(data.markers.size());
  for (const mmr_base::msg::Marker c : data.markers)
  {
    // if (c.confidence >= params->main.min_cone_confidence)
    Node n = Node(c);
    nodes.push_back(n);
  }

  // Update local coordinates of Nodes (makes original local coords unnecessary)
  for (const Node &n : nodes)
  {
    n.updateLocal(wayComputer->getLocalTf());
  }

  // Delaunay triangulation
  TriangleSet triangles = DelaunayTri::compute(nodes);

  // Update the way with the new triangulation
  wayComputer->update(triangles, data.markers[0].header.stamp);

  // Publish loop and write tracklimits to a file
  if (wayComputer->isLoopClosed())
  {
    pubFullCenterLine->publish(wayComputer->getPathCenterLine());
    pubFullBorderLeft->publish(wayComputer->getPathBorderLeft());
    pubFullBorderRight->publish(wayComputer->getPathBorderRight());

    RCLCPP_INFO(rclcpp::get_logger(""), "[urinay] Tanco loop");
    std::string loopDir = params->main.package_path + "/loops";
    mkdir(loopDir.c_str(), 0777);
    wayComputer->writeWayToFile(loopDir + "/loop.unay");
    if (params->main.shutdown_on_loop_closure)
    {
      Time::tock("computation"); // End measuring time
      rclcpp::shutdown();
    }
  }
  // Publish partial
  else
  {
    pubPartialCenterLine->publish(wayComputer->getPathCenterLine());
    pubPartialBorderLeft->publish(wayComputer->getPathBorderLeft());
    pubPartialBorderRight->publish(wayComputer->getPathBorderRight());
  }

  Time::tock("computation"); // End measuring time
}

// Main
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto nh = rclcpp::Node::make_shared("urinay");

  params = new Params(nh);
  wayComputer = new WayComputer(params->wayComputer);
  Visualization::getInstance().init(nh, params->visualization);

  // Subscribers & Publishers
  auto subCones = nh->create_subscription<mmr_base::msg::MarkerArray>(params->main.input_cones_topic, 1, callback_ccat);
  auto subPose = nh->create_subscription<geometry_msgs::msg::PoseStamped>(params->main.input_pose_topic, 1, std::bind(&WayComputer::stateCallback, wayComputer, std::placeholders::_1));

  // publishers
  pubPartialCenterLine = nh->create_publisher<mmr_base::msg::MarkerArray>(params->main.output_partial_center_topic, 1);
  pubPartialBorderLeft = nh->create_publisher<mmr_base::msg::MarkerArray>(params->main.output_partial_left_topic, 1);
  pubPartialBorderRight = nh->create_publisher<mmr_base::msg::MarkerArray>(params->main.output_partial_right_topic, 1);

  pubFullCenterLine = nh->create_publisher<mmr_base::msg::MarkerArray>(params->main.output_full_center_topic, 1);
  pubFullBorderLeft = nh->create_publisher<mmr_base::msg::MarkerArray>(params->main.output_full_left_topic, 1);
  pubFullBorderRight = nh->create_publisher<mmr_base::msg::MarkerArray>(params->main.output_full_right_topic, 1);

  rclcpp::spin(nh);
}
/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

double pose[2] = {0, 0};  // current pose
double pickup[2] = {0, 0};  // current pose

void get_current_pose(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose[0] = msg->pose.pose.position.x;
  pose[1] = msg->pose.pose.position.y;
}

double distToCurrentPos(double goalPos[2])
{
  double dx = goalPos[0] - pose[0];
  double dy = goalPos[1] - pose[1];
  return sqrt(dx*dx + dy*dy);
}

bool reach_pick_up()
{
  if (pose[0] <= -3.6 && pose[1] <=-8.0){
      return true;
  }
}

bool reach_drop_zone()
{
  std::cout << pose[0] << pose[1] << std::endl;
  if (pose[0] >= 1.99 && pose[1] >= 0.08){
      return true;
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  ros::Subscriber pose_sub = n.subscribe("odom", 10, get_current_pose);

  uint32_t shape = visualization_msgs::Marker::CUBE;

  enum State {
    PICKUP,  // going to pick up zon
    CARRY,   // carry to drop zone
    DROP,    // already drop
  } state = PICKUP;

  ROS_INFO("Going to pick up zone ... ");
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "basic_shapes";
  marker.id = 0;

  marker.type = shape;

  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  while (ros::ok())
  {

    ros::spinOnce();

    if (state == State::PICKUP) {
      marker.action = visualization_msgs::Marker::ADD;
      n.getParam("/pick_up_loc/tx", marker.pose.position.x);
      n.getParam("/pick_up_loc/ty", marker.pose.position.y);
      n.getParam("/pick_up_loc/tz", marker.pose.position.z);
      n.getParam("/pick_up_loc/qx", marker.pose.orientation.x);
      n.getParam("/pick_up_loc/qy", marker.pose.orientation.y);
      n.getParam("/pick_up_loc/qz", marker.pose.orientation.z);
      n.getParam("/pick_up_loc/qw", marker.pose.orientation.w);
      pickup[0] = marker.pose.position.x;
      pickup[1] = marker.pose.position.y;
      marker_pub.publish(marker);
      if (reach_pick_up()) {
        sleep(5);
        ROS_INFO("Carrying to drop zone ... ");
        state = CARRY;
      }
    } else if (state == CARRY) {
      marker.action = visualization_msgs::Marker::ADD;
      n.getParam("/drop_off_loc/tx", marker.pose.position.x);
      n.getParam("/drop_off_loc/ty", marker.pose.position.y);
      n.getParam("/drop_off_loc/tz", marker.pose.position.z);
      n.getParam("/drop_off_loc/qx", marker.pose.orientation.x);
      n.getParam("/drop_off_loc/qy", marker.pose.orientation.y);
      n.getParam("/drop_off_loc/qz", marker.pose.orientation.z);
      n.getParam("/drop_off_loc/qw", marker.pose.orientation.w);
      pickup[0] = marker.pose.position.x;
      pickup[1] = marker.pose.position.y;
      std::cout << pickup[0] << " " << pickup[1] << std::endl;
      marker_pub.publish(marker);
      if (reach_drop_zone()) {
        sleep(5);
        ROS_INFO("Reached to drop zone ... ");
        state = DROP;
      }
    } else if (state == DROP){
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      break;
    }
  }
  return 0;
}
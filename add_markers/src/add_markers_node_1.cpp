 
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

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_alone");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::CUBE;

  enum State { PICKUP, HIDE, DROP, } state = PICKUP;
  while (ros::ok())
  {
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

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    if (state == PICKUP) {
      marker.action = visualization_msgs::Marker::ADD;
      n.getParam("/pick_up_loc/tx", marker.pose.position.x);
      n.getParam("/pick_up_loc/ty", marker.pose.position.y);
      n.getParam("/pick_up_loc/tz", marker.pose.position.z);
      n.getParam("/pick_up_loc/qx", marker.pose.orientation.x);
      n.getParam("/pick_up_loc/qy", marker.pose.orientation.y);
      n.getParam("/pick_up_loc/qz", marker.pose.orientation.z);
      n.getParam("/pick_up_loc/qw", marker.pose.orientation.w);
      marker_pub.publish(marker);
      ROS_INFO("Picking up ... ");
      sleep(5);
      state = HIDE;
    } else if (state == HIDE) {
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      ROS_INFO("Hiding ...");
      sleep(5);
      state = DROP;
    } else {
      marker.action = visualization_msgs::Marker::ADD;
      n.getParam("/drop_off_loc/tx", marker.pose.position.x);
      n.getParam("/drop_off_loc/ty", marker.pose.position.y);
      n.getParam("/drop_off_loc/tz", marker.pose.position.z);
      n.getParam("/drop_off_loc/qx", marker.pose.orientation.x);
      n.getParam("/drop_off_loc/qy", marker.pose.orientation.y);
      n.getParam("/drop_off_loc/qz", marker.pose.orientation.z);
      n.getParam("/drop_off_loc/qw", marker.pose.orientation.w);
      marker_pub.publish(marker);
      ROS_INFO("Droping out ...");
      sleep(5);
    }
  }
}
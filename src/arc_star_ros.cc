#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <chrono>

#include "acd/arc_star_detector.h"

typedef std::chrono::high_resolution_clock Clock;

acd::ArcStarDetector detector = acd::ArcStarDetector();
ros::Publisher corner_pub;

void EventMsgCallback(const dvs_msgs::EventArray::ConstPtr &event_msg) {

  const int n_event = event_msg->events.size();
  if (n_event == 0) {return;}

  dvs_msgs::EventArray corner_msg;
  corner_msg.header = event_msg->header;
  corner_msg.width = event_msg->width;
  corner_msg.height = event_msg->height;

  auto t_init = Clock::now();
  for (const auto& e : event_msg->events) {
      // Unroll event array and detect Corners
    if (detector.isCorner(e.ts.toSec(), e.x, e.y, e.polarity)) {
      corner_msg.events.push_back(e);
    }
  }
  auto t_end = Clock::now();

  // Summary from the processed event-package
  // Note: DO NOT use this ROS wrapper for timing benchmarking. Use the stand-alone implementation instead.
  const double elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_init).count();
  const int n_corner = corner_msg.events.size();
  const double percentage_corners = (double(n_corner)/n_event)*100;
  const double time_per_event = elapsed_time/n_event; // Average time to process one event [ns/ev]
  const double event_rate = 1/(time_per_event*1e-3) ; // Average Event Rate [Million ev / s]

  ROS_INFO("Percetange of corners: %.1f%%. Avg. timing: %0.0f ns/ev. Max event rate: %0.2f Mev/s",
           percentage_corners, time_per_event, event_rate);

  // Send detected corner events
  corner_pub.publish(corner_msg);
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "arc_star_ros");
  ros::NodeHandle nh;

//  detector = acd::ArcStarDetector();

  corner_pub = nh.advertise<dvs_msgs::EventArray>("corners", 1);
  ros::Subscriber event_sub = nh.subscribe("events", 0, &EventMsgCallback);

  while (ros::ok()) {ros::spinOnce();} // Preferred over ros::spin() for performance

  return 0;
}


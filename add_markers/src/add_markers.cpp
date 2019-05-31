
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>

uint8_t robot_status = 0;
void subCallback(const std_msgs::UInt8::ConstPtr& message)
{
  ROS_INFO("Message Received!");
  robot_status = message->data;
  return; 
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber location_sub = n.subscribe("/target", 1, subCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE
    marker.type = shape;

    // Cycle between different states of the robot
    switch (robot_status)
    {
    case 0:
      // Set the marker action.  
      marker.action = visualization_msgs::Marker::ADD;

      // Set the initial pose of the marker.  
      marker.pose.position.x = 4;
      marker.pose.position.y = 3;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      break;
    case 1:
      // delete the marker action.  
      marker.action = visualization_msgs::Marker::DELETE;
      break;
    case 2:
      // Set the marker action.  
      marker.action = visualization_msgs::Marker::ADD;

      // Set the initial pose of the marker.  
      marker.pose.position.x = -1.33;
      marker.pose.position.y = -3.8;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      break;
    }

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    r.sleep();
  }
}


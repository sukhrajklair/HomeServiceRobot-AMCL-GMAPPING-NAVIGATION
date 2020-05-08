#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "add_markers/DisplayMarker.h"

class MarkerServeAndPublish
{
private:
	ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::ServiceServer marker_serv;
  
  bool serve_callback(add_markers::DisplayMarker::Request& req, add_markers::DisplayMarker::Response& res)
  {
  	// Set our shape type to be a cube
  	uint32_t shape = visualization_msgs::Marker::CUBE;
  	visualization_msgs::Marker marker;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "marker";
		marker.id = 0;

		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, 		ARROW, and CYLINDER
		marker.type = shape;
		
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time 		specified in the header
		marker.pose.position.x = req.x;
		marker.pose.position.y = req.y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 0.3;
		marker.scale.y = 0.3;
		marker.scale.z = 0.3;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();
		
		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 			    (DELETEALL)
		//check the request to see wether to display the marker or to hide it
 		marker.action = req.display? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
 		
 		marker_pub.publish(marker);
 		
 		res.msg_feedback = "marker set";
 		ROS_INFO_STREAM(res.msg_feedback);
 		
 		return true;
  }
public:
	MarkerServeAndPublish()
	{
		marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
		marker_serv = n.advertiseService("/add_markers/add_marker", &MarkerServeAndPublish::serve_callback, this);
	}

};
int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::Rate r(1);
  MarkerServeAndPublish addMarkers;
  ros::spin();
  return 0;
}

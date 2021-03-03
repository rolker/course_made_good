#include "ros/ros.h"
#include "marine_msgs/CourseMadeGoodStamped.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "project11/utils.h"
#include <deque>

ros::Publisher cmg_pub;

namespace p11 = project11;

p11::LatLongDegrees last_position;

typedef std::pair<ros::Time, p11::LatLongDegrees> TimePoint;
std::deque<TimePoint> last_positions;

/// Time in seconds between positions used to calculate course. Too close in time positions may result in noisy course.
ros::Duration time_span(2.0);

void positionCallback(const geographic_msgs::GeoPointStamped::ConstPtr& message)
{
  p11::LatLongDegrees current_position;
  p11::fromMsg(message->position, current_position);

  while(!last_positions.empty() && (message->header.stamp - last_positions.front().first) > time_span)
    last_positions.pop_front();
    
  if(!last_positions.empty())
  {
    auto azimuth_distance = p11::WGS84::inverse(last_positions.front().second,current_position);
        marine_msgs::CourseMadeGoodStamped cmgs;
        cmgs.header = message->header;
        cmgs.course = p11::AngleDegrees(azimuth_distance.first).value();
        cmg_pub.publish(cmgs);
    }
    else if(last_position[0] != 9999)
    {
        auto azimuth_distance = p11::WGS84::inverse(last_position,current_position);
        marine_msgs::CourseMadeGoodStamped cmgs;
        cmgs.header = message->header;
        cmgs.course = p11::AngleDegrees(azimuth_distance.first).value();
        cmg_pub.publish(cmgs);
    }
    last_position = current_position;
    last_positions.push_back(TimePoint(message->header.stamp,current_position));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "course_made_good");
    ros::NodeHandle n;
    
    last_position[0] = 9999;
    
    ros::Subscriber input = n.subscribe("position",10,&positionCallback);
    
    cmg_pub = n.advertise<marine_msgs::CourseMadeGoodStamped>("cmg",1);
    ros::spin();
    
    return 0;
}

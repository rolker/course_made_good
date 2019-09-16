#include "ros/ros.h"
#include "marine_msgs/CourseMadeGoodStamped.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "project11/gz4d_geo.h"

ros::Publisher cmg_pub;

gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> last_position;

void positionCallback(const geographic_msgs::GeoPointStamped::ConstPtr& message)
{
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> current_position(message->position.latitude,message->position.longitude,0.0);
    if(last_position[0] != 9999)
    {
        auto azimuth_distance = gz4d::geo::WGS84::Ellipsoid::inverse(last_position,current_position);
        marine_msgs::CourseMadeGoodStamped cmgs;
        cmgs.header = message->header;
        cmgs.course = azimuth_distance.first;
        cmg_pub.publish(cmgs);
    }
    last_position = current_position;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "course_made_good");
    ros::NodeHandle n;
    
    last_position[0] = 9999;
    
    ros::Subscriber input = n.subscribe("/position",10,&positionCallback);
    
    cmg_pub = n.advertise<marine_msgs::CourseMadeGoodStamped>("/cmg",1);
    ros::spin();
    
    return 0;
}

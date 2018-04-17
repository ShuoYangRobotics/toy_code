#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void distanceDetect(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //assume only has distance measurement at mirror position
    int numData = msg->ranges.size();
    for (int i = 0; i < numData; i++)
    {
        if (msg -> ranges[i] > 0.3)
            printf("(%d, %f)\n", i, msg->ranges[i]);
    }
    printf("###############################\n");

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoid");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/laser_scan", 50, distanceDetect);

  ros::spin();
  return 0;
}

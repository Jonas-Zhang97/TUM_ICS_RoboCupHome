#include <pick_place/place.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "place_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  Place place;

  place.init();

  ros::Rate loop_rate(10);
  loop_rate.sleep();

  while (ros::ok())
  { 
    place.update();
    loop_rate.sleep();
  }

  ros::shutdown;
  return 0;
}
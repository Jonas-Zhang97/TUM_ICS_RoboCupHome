#include <pick_place/pick.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  Pick pick;

  pick.init();

  ros::Rate loop_rate(10);
  loop_rate.sleep();

  while (ros::ok())
  { 
    pick.update();
    loop_rate.sleep();
  }

  ros::shutdown;
  return 0;
}
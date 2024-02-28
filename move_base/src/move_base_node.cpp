/*
  这里初始化了节点“move_base_node”，Action服务的定义、全局规划器、局部规划器的调用都将在这个节点中进行。
  然后实例化了MoveBase这个类，上述工作以类成员函数的形式定义在这个类中。
  实例化之后，Action开始监听服务请求，并通过ros::spin()传递到Action的回调函数中进行处理。
 */

#include <move_base/move_base.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "move_base_node");
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  move_base::MoveBase move_base( buffer );

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return(0);
}

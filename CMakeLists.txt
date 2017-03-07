cmake_minimum_required(VERSION 2.8.3)

project(interface_rs485)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(DIRECTORY msg FILES SendRS485Msg.msg)

catkin_python_setup()
generate_messages(DEPENDENCIES std_msgs )
catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


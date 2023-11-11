find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)
add_executable(CartoTransferPose_main CartoTransferPose_main.cc)
target_link_libraries(CartoTransferPose_main ${catkin_LIBRARIES})

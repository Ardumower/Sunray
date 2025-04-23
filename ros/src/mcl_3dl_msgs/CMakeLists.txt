cmake_minimum_required(VERSION 2.8.3)
project(mcl_3dl_msgs)

set(MESSAGE_DEPENDS
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS message_generation ${MESSAGE_DEPENDS})

add_service_files(
  FILES
    ResizeParticle.srv
    LoadPCD.srv
)
add_message_files(
  FILES
    Status.msg
)
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDS})
catkin_package(CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDS})

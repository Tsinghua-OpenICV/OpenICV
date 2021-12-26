cmake_minimum_required(VERSION 2.8.3)
project(canbus_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES
  IPC_SCU_1_0x106.msg
  IPC_SCU_2_0x102.msg
  IPC_SCU_3_0x103.msg
  SCU_IPC_1_0x20A.msg
  SCU_IPC_2_0x205.msg
  SCU_IPC_3_0x206.msg
  SCU_IPC_4_0x207.msg
  SCU_IPC_5_0x208.msg
  SCU_IPC_6_0x209.msg
  SCU_IPC_7_0x301.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)


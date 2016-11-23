cmake_minimum_required(VERSION 2.8.3)
project(ur_arm)


# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp std_msgs std_srvs geometry_msgs message_generation)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

add_message_files(
  FILES
  Joints.msg
)

add_service_files(
  FILES
  SetDigitalOut.srv
)

generate_messages()

# Declare catkin package
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ur_arm
   CATKIN_DEPENDS roscpp std_msgs std_srvs geometry_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} )


## Build
# Declare a cpp library
add_library(ur_arm src/ur_arm.cpp src/ur_arm_forkin.cpp)

## Declare a cpp executable
add_executable(ur_arm_node src/ur_arm_node.cpp src/main.cpp)
target_link_libraries(ur_arm_node ur_arm ${catkin_LIBRARIES})
add_dependencies(ur_arm_node ${PROJECT_NAME}_gencpp)



## Install
# Mark executables and/or libraries for installation
install(TARGETS ur_arm ur_arm_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


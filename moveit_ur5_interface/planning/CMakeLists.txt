add_executable(planning src/planning.cpp)
target_link_libraries(planning ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

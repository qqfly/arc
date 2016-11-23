add_executable(nrm_dec src/nrm_dec.cpp)
target_link_libraries(nrm_dec ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS nrm_dec DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

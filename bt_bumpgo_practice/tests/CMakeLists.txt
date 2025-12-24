add_executable(test_single_movement test_single_movement.cpp)
ament_target_dependencies(test_single_movement ${dependencies})
target_link_libraries(test_single_movement ${ZMQ_LIBRARIES})

install(TARGETS
    test_single_movement
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
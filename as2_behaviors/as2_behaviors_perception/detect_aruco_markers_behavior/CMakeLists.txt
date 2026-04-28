set(EXECUTABLE_NAME detect_aruco_markers_behavior)

# Find dependencies
set(EXECUTABLE_DEPENDENCIES
  sensor_msgs
)

foreach(DEPENDENCY ${EXECUTABLE_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
  include/${EXECUTABLE_NAME}
)

set(SOURCE_CPP_FILES
  src/${EXECUTABLE_NAME}.cpp
  src/${EXECUTABLE_NAME}_node.cpp
)

add_executable(${EXECUTABLE_NAME}_node ${SOURCE_CPP_FILES})
target_link_libraries(${EXECUTABLE_NAME}_node ${OPENCV_LIBS})
ament_target_dependencies(${EXECUTABLE_NAME}_node ${PROJECT_DEPENDENCIES} ${EXECUTABLE_DEPENDENCIES})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/${EXECUTABLE_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${EXECUTABLE_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${EXECUTABLE_DEPENDENCIES})
ament_export_include_directories(include)


set(EXECUTABLE_NAME2 detect_aruco_markers_behavior_pub_landingpad)

# Find dependencies
set(EXECUTABLE_DEPENDENCIES2
  sensor_msgs
)

foreach(DEPENDENCY ${EXECUTABLE_DEPENDENCIES2})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
  include/${EXECUTABLE_NAME2}
)

set(SOURCE_CPP_FILES
  src/${EXECUTABLE_NAME2}.cpp
  src/${EXECUTABLE_NAME2}_node.cpp
)

add_executable(${EXECUTABLE_NAME2}_node ${SOURCE_CPP_FILES})
target_link_libraries(${EXECUTABLE_NAME2}_node ${OPENCV_LIBS})
ament_target_dependencies(${EXECUTABLE_NAME2}_node ${PROJECT_DEPENDENCIES} ${EXECUTABLE_DEPENDENCIES2})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/${EXECUTABLE_NAME2})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${EXECUTABLE_NAME2}_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${EXECUTABLE_DEPENDENCIES2})
ament_export_include_directories(include)
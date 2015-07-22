cmake_minimum_required(VERSION 2.8.3)
project(ardrone_gui)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rqt_gui rqt_gui_cpp rospy OpenCV REQUIRED cv_bridge message_filters)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIRS})
MESSAGE(${CMAKE_CURRENT_BINARY_DIR}/../devel/lib)

set(rqt_my_plugin_SRCS
    src/my_namespace/my_plugin.cpp
)

set(rqt_my_plugin_HDRS
    include/my_namespace/my_plugin.h
)

set(rqt_my_plugin_UIS
	src/my_namespace/my_plugin.ui
)

set(rqt_my_plugin_INCLUDE_DIRECTORIES
    include/
    ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
   INCLUDE_DIRS ${rqt_my_plugin_INCLUDE_DIRECTORIES}
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp std_msgs rqt_gui rqt_gui_cpp
)

qt4_wrap_cpp(rqt_my_plugin_MOCS ${rqt_my_plugin_HDRS})
qt4_wrap_ui(rqt_my_plugin_UIS_H ${rqt_my_plugin_UIS})

###########
## Build ##
###########

include_directories(
  ${rqt_my_plugin_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(
  ${PROJECT_NAME} 
  ${rqt_my_plugin_SRCS} 
  ${rqt_my_plugin_MOCS} 
  ${rqt_my_plugin_UIS_H}
)

add_executable(faces src/faces.cpp)
target_link_libraries(faces ${catkin_LIBRARIES} ${OpenCV_LIBS})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

MESSAGE(${QT_QTCORE_LIBRARY})

LINK_DIRECTORIES(/usr/lib/i386-linux-gnu)

target_link_libraries(
  ardrone_gui
  ${catkin_LIBRARIES} 
  ${QT_QTCORE_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
)
  
find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

#############
## Install ##
#############

install(FILES
	plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
   scripts/ardrone_gui
   scripts/joy.py
   scripts/test.py
   scripts/controller.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
MESSAGE(${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
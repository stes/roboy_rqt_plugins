cmake_minimum_required(VERSION 2.8.3)
project( pabiroboy_joint_control )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS roscpp roslint rqt_gui rqt_gui_cpp std_msgs roboy_communication_middleware )
catkin_package(CATKIN_DEPENDS roboy_communication_middleware)

include_directories( include qcustomplot)

## System dependencies are found with CMake's conventions

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

roslint_cpp()

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs roboy_communication_middleware)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(pabiroboy_joint_control_MOCS include/pabiroboy_joint_control/pabiroboy_joint_control.hpp qcustomplot/qcustomplot.h)
  qt5_wrap_ui(pabiroboy_joint_control_UIS_H src/pabiroboy_joint_control.ui)
else()
  qt4_wrap_cpp(pabiroboy_joint_control_MOCS include/pabiroboy_joint_control/pabiroboy_joint_control.hpp qcustomplot/qcustomplot.h)
  qt4_wrap_ui(pabiroboy_joint_control_UIS_H src/pabiroboy_joint_control.ui)
endif()

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

add_library( pabiroboy_joint_control src/pabiroboy_joint_control.cpp
        ${pabiroboy_joint_control_MOCS} ${pabiroboy_joint_control_UIS_H}
        qcustomplot/qcustomplot.cpp)

add_dependencies(pabiroboy_joint_control roboy_communication_middleware_gencpp)

target_link_libraries(pabiroboy_joint_control ${catkin_LIBRARIES} )

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(pabiroboy_joint_control Qt5::Widgets)
else()
  target_link_libraries(pabiroboy_joint_control ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#############
## Install ##
#############

install(TARGETS pabiroboy_joint_control
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES
        package.xml
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
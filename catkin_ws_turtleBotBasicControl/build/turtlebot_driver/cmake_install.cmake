# Install script for directory: /home/tul1/Desktop/TurtleBotBasicControl/catkin_ws_turtleBotBasicControl/src/turtlebot_driver

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/tul1/Desktop/TurtleBotBasicControl/catkin_ws_turtleBotBasicControl/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/tul1/Desktop/TurtleBotBasicControl/catkin_ws_turtleBotBasicControl/build/turtlebot_driver/catkin_generated/installspace/turtlebot_driver.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_driver/cmake" TYPE FILE FILES
    "/home/tul1/Desktop/TurtleBotBasicControl/catkin_ws_turtleBotBasicControl/build/turtlebot_driver/catkin_generated/installspace/turtlebot_driverConfig.cmake"
    "/home/tul1/Desktop/TurtleBotBasicControl/catkin_ws_turtleBotBasicControl/build/turtlebot_driver/catkin_generated/installspace/turtlebot_driverConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_driver" TYPE FILE FILES "/home/tul1/Desktop/TurtleBotBasicControl/catkin_ws_turtleBotBasicControl/src/turtlebot_driver/package.xml")
endif()


# Install script for directory: /home/sfbot/catkin_ws/src/sfbot/sfbot_imu/mpu6050_serial_to_imu

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sfbot/catkin_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sfbot/catkin_ws/build/sfbot/sfbot_imu/mpu6050_serial_to_imu/catkin_generated/installspace/mpu6050_serial_to_imu.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mpu6050_serial_to_imu/cmake" TYPE FILE FILES
    "/home/sfbot/catkin_ws/build/sfbot/sfbot_imu/mpu6050_serial_to_imu/catkin_generated/installspace/mpu6050_serial_to_imuConfig.cmake"
    "/home/sfbot/catkin_ws/build/sfbot/sfbot_imu/mpu6050_serial_to_imu/catkin_generated/installspace/mpu6050_serial_to_imuConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mpu6050_serial_to_imu" TYPE FILE FILES "/home/sfbot/catkin_ws/src/sfbot/sfbot_imu/mpu6050_serial_to_imu/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mpu6050_serial_to_imu/mpu6050_serial_to_imu_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mpu6050_serial_to_imu/mpu6050_serial_to_imu_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mpu6050_serial_to_imu/mpu6050_serial_to_imu_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mpu6050_serial_to_imu" TYPE EXECUTABLE FILES "/home/sfbot/catkin_ws/devel/lib/mpu6050_serial_to_imu/mpu6050_serial_to_imu_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mpu6050_serial_to_imu/mpu6050_serial_to_imu_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mpu6050_serial_to_imu/mpu6050_serial_to_imu_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mpu6050_serial_to_imu/mpu6050_serial_to_imu_node"
         OLD_RPATH "/opt/ros/kinetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mpu6050_serial_to_imu/mpu6050_serial_to_imu_node")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mpu6050_serial_to_imu" TYPE DIRECTORY FILES "/home/sfbot/catkin_ws/src/sfbot/sfbot_imu/mpu6050_serial_to_imu/launch")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mpu6050_serial_to_imu" TYPE DIRECTORY FILES "/home/sfbot/catkin_ws/src/sfbot/sfbot_imu/mpu6050_serial_to_imu/rviz")
endif()


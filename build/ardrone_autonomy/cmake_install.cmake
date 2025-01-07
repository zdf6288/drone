# Install script for directory: /home/zhang/drone_workspace/src/ardrone_autonomy

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zhang/drone_workspace/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy/srv" TYPE FILE FILES
    "/home/zhang/drone_workspace/src/ardrone_autonomy/srv/CamSelect.srv"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/srv/FlightAnim.srv"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/srv/LedAnim.srv"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/srv/RecordEnable.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy/msg" TYPE FILE FILES
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/matrix33.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_adc_data_frame.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_altitude.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_demo.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_euler_angles.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_games.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_gyros_offsets.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_hdvideo_stream.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_kalman_pressure.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_magneto.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/Navdata.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_phys_measures.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_pressure_raw.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_pwm.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_raw_measures.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_rc_references.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_references.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_time.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_trackers_send.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_trims.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_video_stream.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_vision_detect.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_vision.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_vision_of.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_vision_perf.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_vision_raw.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_watchdog.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_wifi.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_wind_speed.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/navdata_zimmu_3000.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/vector21.msg"
    "/home/zhang/drone_workspace/src/ardrone_autonomy/msg/vector31.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy/cmake" TYPE FILE FILES "/home/zhang/drone_workspace/build/ardrone_autonomy/catkin_generated/installspace/ardrone_autonomy-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zhang/drone_workspace/devel/include/ardrone_autonomy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/zhang/drone_workspace/devel/share/roseus/ros/ardrone_autonomy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zhang/drone_workspace/devel/share/common-lisp/ros/ardrone_autonomy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/zhang/drone_workspace/devel/share/gennodejs/ros/ardrone_autonomy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/zhang/drone_workspace/devel/lib/python3/dist-packages/ardrone_autonomy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/zhang/drone_workspace/devel/lib/python3/dist-packages/ardrone_autonomy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zhang/drone_workspace/build/ardrone_autonomy/catkin_generated/installspace/ardrone_autonomy.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy/cmake" TYPE FILE FILES "/home/zhang/drone_workspace/build/ardrone_autonomy/catkin_generated/installspace/ardrone_autonomy-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy/cmake" TYPE FILE FILES
    "/home/zhang/drone_workspace/build/ardrone_autonomy/catkin_generated/installspace/ardrone_autonomyConfig.cmake"
    "/home/zhang/drone_workspace/build/ardrone_autonomy/catkin_generated/installspace/ardrone_autonomyConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy" TYPE FILE FILES "/home/zhang/drone_workspace/src/ardrone_autonomy/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy" TYPE EXECUTABLE FILES "/home/zhang/drone_workspace/devel/lib/ardrone_autonomy/ardrone_driver")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver"
         OLD_RPATH "/home/zhang/drone_workspace/devel/lib/ardrone:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy/launch" TYPE DIRECTORY FILES "/home/zhang/drone_workspace/src/ardrone_autonomy/launch/" REGEX "/\\.git$" EXCLUDE)
endif()


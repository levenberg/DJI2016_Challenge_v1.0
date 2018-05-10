# Install script for directory: /home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yangpc/workspace/DJI2016_Challenge_v1.0/install")
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
  include("/home/yangpc/workspace/DJI2016_Challenge_v1.0/build/dji_sdk/catkin_generated/safe_execute_install.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/A3GPS.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/A3RTK.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/Acceleration.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/AttitudeQuaternion.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/Compass.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/FlightControlInfo.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/Gimbal.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/GlobalPosition.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/LocalPosition.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/PowerStatus.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/RCChannels.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/Velocity.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/Waypoint.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/WaypointList.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/TransparentTransmissionData.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/TimeStamp.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionPushInfo.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionWaypointAction.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionWaypoint.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionWaypointTask.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionHotpointTask.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionFollowmeTask.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionFollowmeTarget.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionStatusWaypoint.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionStatusHotpoint.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionStatusFollowme.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionStatusOther.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionEventWpUpload.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionEventWpAction.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/MissionEventWpReach.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/DetectionPoints.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/msg/Reldist.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/srv" TYPE FILE FILES
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/Activation.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/AttitudeControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/CameraActionControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/DroneTaskControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/GimbalAngleControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/GimbalSpeedControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/GlobalPositionControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/LocalPositionControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/SDKPermissionControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/SendDataToRemoteDevice.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/VelocityControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/VersionCheck.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/DroneArmControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/SyncFlagControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MessageFrequencyControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/VirtualRCEnableControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/VirtualRCDataControl.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionStart.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionPause.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionCancel.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionWpUpload.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionWpSetSpeed.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionWpGetSpeed.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionWpDownload.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionHpUpload.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionHpDownload.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionHpSetSpeed.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionHpSetRadius.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionHpResetYaw.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionFmUpload.srv"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/srv/MissionFmSetTarget.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/action" TYPE FILE FILES
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/action/GlobalPositionNavigation.action"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/action/LocalPositionNavigation.action"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/action/WaypointNavigation.action"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/action/DroneTask.action"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/GlobalPositionNavigationAction.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/GlobalPositionNavigationActionGoal.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/GlobalPositionNavigationActionResult.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/GlobalPositionNavigationActionFeedback.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/GlobalPositionNavigationGoal.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/GlobalPositionNavigationResult.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/GlobalPositionNavigationFeedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/LocalPositionNavigationAction.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/LocalPositionNavigationActionGoal.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/LocalPositionNavigationActionResult.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/LocalPositionNavigationActionFeedback.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/LocalPositionNavigationGoal.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/LocalPositionNavigationResult.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/LocalPositionNavigationFeedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/WaypointNavigationAction.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/WaypointNavigationActionGoal.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/WaypointNavigationActionResult.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/WaypointNavigationActionFeedback.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/WaypointNavigationGoal.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/WaypointNavigationResult.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/WaypointNavigationFeedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/DroneTaskAction.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/DroneTaskActionGoal.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/DroneTaskActionResult.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/DroneTaskActionFeedback.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/DroneTaskGoal.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/DroneTaskResult.msg"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg/DroneTaskFeedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/cmake" TYPE FILE FILES "/home/yangpc/workspace/DJI2016_Challenge_v1.0/build/dji_sdk/catkin_generated/installspace/dji_sdk-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/include/dji_sdk")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/share/common-lisp/ros/dji_sdk")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/lib/python2.7/dist-packages/dji_sdk")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/lib/python2.7/dist-packages/dji_sdk" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/lib/python2.7/dist-packages/dji_sdk" FILES_MATCHING REGEX "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/lib/python2.7/dist-packages/dji_sdk/.+/__init__.pyc?$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yangpc/workspace/DJI2016_Challenge_v1.0/build/dji_sdk/catkin_generated/installspace/dji_sdk.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/cmake" TYPE FILE FILES "/home/yangpc/workspace/DJI2016_Challenge_v1.0/build/dji_sdk/catkin_generated/installspace/dji_sdk-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/cmake" TYPE FILE FILES
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/build/dji_sdk/catkin_generated/installspace/dji_sdkConfig.cmake"
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/build/dji_sdk/catkin_generated/installspace/dji_sdkConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk" TYPE FILE FILES "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dji_sdk" TYPE DIRECTORY FILES "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/include/dji_sdk/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dji_sdk" TYPE EXECUTABLE FILES "/home/yangpc/workspace/DJI2016_Challenge_v1.0/devel/lib/dji_sdk/dji_sdk_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/launch" TYPE DIRECTORY FILES "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/launch")
endif()


# Install script for directory: /root/Documents/roswork/DJI2016_Challenge/src/dji_sdk

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/root/Documents/roswork/DJI2016_Challenge/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  INCLUDE("/root/Documents/roswork/DJI2016_Challenge/build/dji_sdk/catkin_generated/safe_execute_install.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/A3GPS.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/A3RTK.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/Acceleration.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/AttitudeQuaternion.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/Compass.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/FlightControlInfo.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/Gimbal.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/GlobalPosition.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/LocalPosition.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/PowerStatus.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/RCChannels.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/Velocity.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/Waypoint.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/WaypointList.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/TransparentTransmissionData.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/TimeStamp.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionPushInfo.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionWaypointAction.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionWaypoint.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionWaypointTask.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionHotpointTask.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionFollowmeTask.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionFollowmeTarget.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionStatusWaypoint.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionStatusHotpoint.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionStatusFollowme.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionStatusOther.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionEventWpUpload.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionEventWpAction.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/MissionEventWpReach.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/DetectionPoints.msg"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg/Reldist.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/srv" TYPE FILE FILES
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/Activation.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/AttitudeControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/CameraActionControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/DroneTaskControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/GimbalAngleControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/GimbalSpeedControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/GlobalPositionControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/LocalPositionControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/SDKPermissionControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/SendDataToRemoteDevice.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/VelocityControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/VersionCheck.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/DroneArmControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/SyncFlagControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MessageFrequencyControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/VirtualRCEnableControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/VirtualRCDataControl.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionStart.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionPause.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionCancel.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionWpUpload.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionWpSetSpeed.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionWpGetSpeed.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionWpDownload.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionHpUpload.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionHpDownload.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionHpSetSpeed.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionHpSetRadius.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionHpResetYaw.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionFmUpload.srv"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/srv/MissionFmSetTarget.srv"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/action" TYPE FILE FILES
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/action/GlobalPositionNavigation.action"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/action/LocalPositionNavigation.action"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/action/WaypointNavigation.action"
    "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/action/DroneTask.action"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/GlobalPositionNavigationAction.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/GlobalPositionNavigationActionGoal.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/GlobalPositionNavigationActionResult.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/GlobalPositionNavigationActionFeedback.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/GlobalPositionNavigationGoal.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/GlobalPositionNavigationResult.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/GlobalPositionNavigationFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/LocalPositionNavigationAction.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/LocalPositionNavigationActionGoal.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/LocalPositionNavigationActionResult.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/LocalPositionNavigationActionFeedback.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/LocalPositionNavigationGoal.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/LocalPositionNavigationResult.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/LocalPositionNavigationFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/WaypointNavigationAction.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/WaypointNavigationActionGoal.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/WaypointNavigationActionResult.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/WaypointNavigationActionFeedback.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/WaypointNavigationGoal.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/WaypointNavigationResult.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/WaypointNavigationFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/DroneTaskAction.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/DroneTaskActionGoal.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/DroneTaskActionResult.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/DroneTaskActionFeedback.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/DroneTaskGoal.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/DroneTaskResult.msg"
    "/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg/DroneTaskFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/cmake" TYPE FILE FILES "/root/Documents/roswork/DJI2016_Challenge/build/dji_sdk/catkin_generated/installspace/dji_sdk-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/root/Documents/roswork/DJI2016_Challenge/devel/include/dji_sdk")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/root/Documents/roswork/DJI2016_Challenge/devel/share/common-lisp/ros/dji_sdk")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/root/Documents/roswork/DJI2016_Challenge/devel/lib/python2.7/dist-packages/dji_sdk")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/root/Documents/roswork/DJI2016_Challenge/devel/lib/python2.7/dist-packages/dji_sdk" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/root/Documents/roswork/DJI2016_Challenge/devel/lib/python2.7/dist-packages/dji_sdk" FILES_MATCHING REGEX "/root/Documents/roswork/DJI2016_Challenge/devel/lib/python2.7/dist-packages/dji_sdk/.+/__init__.pyc?$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/Documents/roswork/DJI2016_Challenge/build/dji_sdk/catkin_generated/installspace/dji_sdk.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/cmake" TYPE FILE FILES "/root/Documents/roswork/DJI2016_Challenge/build/dji_sdk/catkin_generated/installspace/dji_sdk-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/cmake" TYPE FILE FILES
    "/root/Documents/roswork/DJI2016_Challenge/build/dji_sdk/catkin_generated/installspace/dji_sdkConfig.cmake"
    "/root/Documents/roswork/DJI2016_Challenge/build/dji_sdk/catkin_generated/installspace/dji_sdkConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk" TYPE FILE FILES "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dji_sdk" TYPE DIRECTORY FILES "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/include/dji_sdk/")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node"
         RPATH "")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dji_sdk" TYPE EXECUTABLE FILES "/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk/dji_sdk_node")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/launch" TYPE DIRECTORY FILES "/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/launch")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")


execute_process(COMMAND "/root/Documents/roswork/DJI2016_Challenge_v1.0/build/dji_sdk/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/root/Documents/roswork/DJI2016_Challenge_v1.0/build/dji_sdk/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

# Install script for directory: /home/muneeb/my-robotic-arm/arm_ws/src/sim_ros2_interface

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/muneeb/my-robotic-arm/arm_ws/install/sim_ros2_interface")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/lua/simROS2-typecheck.lua")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/lua" TYPE FILE RENAME "simROS2-typecheck.lua" FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/generated/typecheck.lua")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/lua/simROS2.lua")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/lua" TYPE FILE RENAME "simROS2.lua" FILES "/home/muneeb/my-robotic-arm/arm_ws/src/sim_ros2_interface/lua/simROS2.lua")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/manual/en/simROS2.htm")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/manual/en" TYPE FILE RENAME "simROS2.htm" FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/generated/reference.html")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/manual/index/simROS2.json")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/manual/index" TYPE FILE RENAME "simROS2.json" FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/generated/index.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/lua/simROS2-ce.lua")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/lua" TYPE FILE RENAME "simROS2-ce.lua" FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/generated/ce.lua")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/libsimROS2.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/libsimROS2.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/libsimROS2.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/libsimROS2.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04" TYPE SHARED_LIBRARY FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/libsimROS2.so")
  if(EXISTS "$ENV{DESTDIR}/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/libsimROS2.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/libsimROS2.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/libsimROS2.so"
         OLD_RPATH "/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib:/home/muneeb/my-robotic-arm/arm_ws/install/custom_message/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/muneeb/Apps/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/libsimROS2.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/CMakeFiles/simROS2.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/sim_ros2_interface")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/sim_ros2_interface")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sim_ros2_interface/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sim_ros2_interface/environment" TYPE FILE FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sim_ros2_interface/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sim_ros2_interface/environment" TYPE FILE FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sim_ros2_interface" TYPE FILE FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sim_ros2_interface" TYPE FILE FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sim_ros2_interface" TYPE FILE FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sim_ros2_interface" TYPE FILE FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sim_ros2_interface" TYPE FILE FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/ament_cmake_index/share/ament_index/resource_index/packages/sim_ros2_interface")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sim_ros2_interface/cmake" TYPE FILE FILES
    "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/ament_cmake_core/sim_ros2_interfaceConfig.cmake"
    "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/ament_cmake_core/sim_ros2_interfaceConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sim_ros2_interface" TYPE FILE FILES "/home/muneeb/my-robotic-arm/arm_ws/src/sim_ros2_interface/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
  file(WRITE "/home/muneeb/my-robotic-arm/arm_ws/build/sim_ros2_interface/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()

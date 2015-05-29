cmake_minimum_required(VERSION 2.8.3)
project(hcrl_simulator)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    roscpp
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem program_options)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    Wrenches.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES controlit
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


# #set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# #set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
# rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# include (FindPkgConfig)
# if (PKG_CONFIG_FOUND)
#   pkg_check_modules(GAZEBO gazebo REQUIRED)
# else()
#   message(FATAL_ERROR "pkg-config is required; please install it")
# endif()

# message(STATUS "Found gazebo:")
# message(STATUS "  include: ${GAZEBO_INCLUDE_DIRS}")
# message(STATUS "  library: ${GAZEBO_LIBRARY_DIRS}")

include_directories(
    ${hcrl_simulator_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# link_directories(${GAZEBO_LIBRARY_DIRS})

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

get_cmake_property(_variableNames VARIABLES)
foreach (_variableName ${_variableNames})
    message(STATUS "+++ ${_variableName}=${${_variableName}}")
endforeach()

# include_directories(${roscpp_INCLUDE_DIRS})
# link_directories(${roscpp_LIBRARY_DIRS})
# include_directories(${rosconsole_INCLUDE_DIRS})
# link_directories(${rosconsole_LIBRARY_DIRS})

add_library(PinPlugin 
    gazebo_plugins/src/PinPlugin.cpp
)

target_link_libraries(PinPlugin 
    ${GAZEBO_LIBRARIES}
)

add_library(WrenchMeasurementPlugin SHARED 
    gazebo_plugins/src/WrenchMeasurementPlugin.cpp
)

add_dependencies(WrenchMeasurementPlugin
    hcrl_simulator_generate_messages_cpp
)

target_link_libraries(WrenchMeasurementPlugin 
    ${GAZEBO_LIBRARIES} 
    ${Boost_LIBRARIES}
)

# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)


# Declare the name of the CMake Project
project(unit)
set(DCMAKE_BUILD_TYPE=Release)
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++0x  -fmessage-length=0 ")


#set(CMAKE_BUILD_TYPE Release)
# Find and get all the information about the roscpp package
find_package(roscpp REQUIRED)
find_package(utils REQUIRED)
find_package(main REQUIRED)
find_package(catkin REQUIRED rospy std_msgs genmsg  message_generation)
FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )
#FIND_PACKAGE( drawstuff REQUIRED )
#generate_messages(DEPENDENCIES std_msgs)




add_service_files(
DIRECTORY srv
  FILES
  from_robot.srv
  for_double.srv
  for_feedback.srv
)

generate_messages(DEPENDENCIES std_msgs)

# Declare this project as a catkin package and export the necessary build flags
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES unit
  CATKIN_DEPENDS roscpp
  )



# Add the local headers and the headers from roscpp
include_directories( include
  ${catkin_INCLUDE_DIRS}
)
include_directories(include ${roscpp_INCLUDE_DIRS}  ${utils_INCLUDE_DIRS} ${main_INCLUDE_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/eigen")
include_directories("/home/rokon/ode-0.12/ode/src/joints")
include_directories("/usr/local/include")
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

link_directories("/usr/local/lib")
link_directories("home/rokon/ode-0.12/drawstuff/src")
# Define a library target called hello_world
add_executable(unit unit_working.cpp )
add_executable(sim new_sim.cpp )
add_executable(collision collision.cpp )
add_executable(fresh fresh.cpp )
add_executable(fresh_pbp fresh_pbp.cpp )
add_executable(fresh_again fresh_again.cpp )
add_executable(fresh_double fresh_double.cpp )
#add_executable(fresh_obs fresh_double_obs.cpp )
add_executable(fresh_working working_rel_double.cpp )

add_executable(single_working single_workin_y.cpp )

add_executable(fresh_short fresh_shorter.cpp )
add_executable(fresh_double_short working_double_shorter.cpp )
add_executable(fresh_feedback fresh_feedback.cpp )
add_executable(fresh_rolling fresh_rolling.cpp )

#add_executable(unit_corrected unit_double_corrected.cpp )


target_link_libraries(unit ${roscpp_LIBRARIES} utils ode main)
target_link_libraries(sim ${roscpp_LIBRARIES} utils ode main drawstuff GL GLU glut)
target_link_libraries(collision ${roscpp_LIBRARIES} utils ode main drawstuff GL GLU glut)
target_link_libraries(fresh ${roscpp_LIBRARIES} utils ode main drawstuff GL GLU glut)
target_link_libraries(fresh_pbp ${roscpp_LIBRARIES} utils ode main drawstuff GL GLU glut)
target_link_libraries(fresh_again ${roscpp_LIBRARIES} utils ode main drawstuff GL GLU glut)
target_link_libraries(fresh_double ${roscpp_LIBRARIES} utils ode main drawstuff GL GLU glut)
#target_link_libraries(fresh_obs ${roscpp_LIBRARIES} utils ode main drawstuff GL GLU glut)
target_link_libraries(fresh_working ${roscpp_LIBRARIES} utils ode main drawstuff GL GLU glut)

target_link_libraries(single_working ${roscpp_LIBRARIES} utils ode main drawstuff GL GLU glut)

target_link_libraries(fresh_short ${roscpp_LIBRARIES} utils ode main drawstuff GL GLU glut)
target_link_libraries(fresh_double_short ${roscpp_LIBRARIES} utils ode main drawstuff GL GLU glut)

target_link_libraries(fresh_feedback ${roscpp_LIBRARIES} utils ode main drawstuff GL GLU glut)
target_link_libraries(fresh_rolling ${roscpp_LIBRARIES} utils ode main drawstuff GL GLU glut)





#target_link_libraries(unit_corrected ${roscpp_LIBRARIES} utils ode main)



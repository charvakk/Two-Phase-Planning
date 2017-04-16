cmake_minimum_required (VERSION 2.6.0)
add_compile_options(-std=c++11)
project(2phase_planning)
find_package(OpenRAVE 0.9 REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(2phase_planning SHARED 2phase_planning.cpp 2phase_planning.h)
set_target_properties(2phase_planning PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(2phase_planning ${OpenRAVE_LIBRARIES})
install(TARGETS 2phase_planning DESTINATION .)

cmake_minimum_required(VERSION 2.8.3)
project(rhoban_model_learning)

find_package(catkin REQUIRED COMPONENTS
  rhoban_bbo
  model
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rhoban_model_learning
  CATKIN_DEPENDS rhoban_bbo model
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

# Directories containing sources files
set(DIRECTORIES
  src/rhoban_model_learning/
  src/rhoban_model_learning/models
  src/rhoban_model_learning/humanoid_models
  )

# Build ALL_SOURCES
set (SOURCES)
set (ALL_SOURCES)
set (PREFIXED_SOURCES)
foreach (DIRECTORY ${DIRECTORIES})
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/${DIRECTORY}")
    include (${DIRECTORY}/sources.cmake)

    foreach (SOURCE ${SOURCES})
        set (PREFIXED_SOURCES ${PREFIXED_SOURCES} ${DIRECTORY}/${SOURCE})
    endforeach (SOURCE)

    set (ALL_SOURCES ${ALL_SOURCES} ${PREFIXED_SOURCES})
endforeach (DIRECTORY)

# Declare a C++ library
add_library(rhoban_model_learning
  ${ALL_SOURCES}
  )

target_link_libraries(rhoban_model_learning
  ${catkin_LIBRARIES}
  )

# Binaries
add_executable(test_simple_angular_model src/tests/test_simple_angular_model.cpp)

target_link_libraries(test_simple_angular_model ${catkin_LIBRARIES} rhoban_model_learning)
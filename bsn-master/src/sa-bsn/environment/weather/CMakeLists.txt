CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(weather)

add_compile_options(-std=c++11)

###########################################################################
## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg messages archlib libbsn)

###########################################################################
# Export catkin package.
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS messages message_runtime archlib libbsn
)

###########################################################################
# Set catkin directory.
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Build this project.
FILE(GLOB ${PROJECT_NAME}-src "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

SET(weather-src "${CMAKE_CURRENT_SOURCE_DIR}/src/WeatherModule.cpp")
ADD_EXECUTABLE (weather  "${CMAKE_CURRENT_SOURCE_DIR}/apps/weather.cpp" ${${PROJECT_NAME}-src} ${weather-src})
TARGET_LINK_LIBRARIES (weather ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(weather services_generate_messages_cpp)
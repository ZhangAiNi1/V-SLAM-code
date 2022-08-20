PROJECT(example)

FIND_PACKAGE(Eigen3)
FIND_PACKAGE(Pangolin)

INCLUDE_DIRECTORIES(${Eigen3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${Pangolin_INCLUDE_DIR})

ADD_EXECUTABLE(coordinateTransform coordinateTransform.cpp)
ADD_EXECUTABLE(plotTrajectory plotTrajectory.cpp)

TARGET_LINK_LIBRARIES(plotTrajectory ${Pangolin_LIBRARY})
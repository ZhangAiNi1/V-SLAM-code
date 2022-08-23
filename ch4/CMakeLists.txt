PROJECT(usesophus)

FIND_PACKAGE(Eigen3)
FIND_PACKAGE(Sophus)
FIND_PACKAGE(Pangolin)

INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIR} )
INCLUDE_DIRECTORIES(${Pangolin_INCLUDE_DIR})

ADD_EXECUTABLE(useSophus useSophus.cpp)
ADD_EXECUTABLE(trajectoryError trajectoryError.cpp)

target_link_libraries(trajectoryError ${Pangolin_LIBRARY}) 
target_link_libraries(useSophus  Sophus::Sophus) 
target_link_libraries(trajectoryError Sophus::Sophus) 

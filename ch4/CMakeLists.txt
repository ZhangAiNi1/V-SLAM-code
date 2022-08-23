PROJECT(usesophus)

FIND_PACKAGE(Eigen3)
FIND_PACKAGE(Sophus)

INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIR})

ADD_EXECUTABLE(useSophus useSophus.cpp)
target_link_libraries(useSophus Sophus::Sophus) 

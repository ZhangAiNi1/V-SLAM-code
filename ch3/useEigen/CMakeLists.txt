project(useEigen)

# 添加Eigen头文件
FIND_PACKAGE(Eigen3)
IF(Eigen3_FOUND)
          include_directories(${Eigen3_INCLUDE_DIR})
ELSE(Eigen3_FOUND)
          MESSAGE(FATAL_ERROR "Eigen3 library not found")
ENDIF(Eigen3_FOUND)

add_executable(eigenMatrix eigenMatrix.cpp)


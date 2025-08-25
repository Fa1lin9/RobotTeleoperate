############################################################
#   Find Cutting For Welding dependant packages for user tangtang 4090 Windows

#   The dependant packages include
#   --  Qt 5.15.5
#   --  Boost 1.78.0
#   --  Eigen
#   --  libfranka
#   --  pinocchio
#   --  osqp-eigen
#   Try to find each packages with an imported target
#   Written by djr in 2025.1
############################################################

# #######################################################
# #                     Find Eigen                      #
# #######################################################

find_package(Eigen3 REQUIRED PATHS E:/DJR/Eigen3/cmake )

if (Eigen3_FOUND)
        message( STATUS "Eigen3 is found!" )
#        get_target_property(Eigen_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
#        message( STATUS "Eigen_DIR is ${Eigen_DIR}. ")
        cmake_print_properties( TARGETS Eigen3::Eigen
            PROPERTIES
                INCLUDE_DIRECTORIES
                INTERFACE_INCLUDE_DIRECTORIES
                IMPORTED_LOCATION_DEBUG
                IMPORTED_IMPLIB_DEBUG
                IMPORTED_LOCATION_RELEASE
                IMPORTED_IMPLIB_RELEASE
                IMPORTED_CONFIGURATIONS
                IMPORTED_LOCATION
                IMPORTED_IMPLIB
                )
endif()


# #######################################################
# #                     Find Boost                      #
# #           Boost should be found before pinocchio    #
# #######################################################
set( CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE )
# set(BOOST_ROOT "E:/DJR/VS2019Prebuild/boost_1_78_0")
# set(Boost_NO_SYSTEM_PATHS ON)  # 禁止 CMake 查找系统路径中的 Boost
set(Boost_Components locale date_time filesystem timer regex thread serialization system program_options )
find_package(Boost REQUIRED COMPONENTS ${Boost_Components} )

if(Boost_FOUND)
    message( STATUS "Boost is found!")
    cmake_print_properties( TARGETS Boost::headers
        PROPERTIES
            INCLUDE_DIRECTORIES
            INTERFACE_INCLUDE_DIRECTORIES
            IMPORTED_LOCATION_DEBUG
            IMPORTED_IMPLIB_DEBUG
            IMPORTED_LOCATION_RELEASE
            IMPORTED_IMPLIB_RELEASE
            IMPORTED_CONFIGURATIONS
            IMPORTED_LOCATION
            IMPORTED_IMPLIB
            )

    cmake_print_properties( TARGETS Boost::serialization
        PROPERTIES
            INCLUDE_DIRECTORIES
            INTERFACE_INCLUDE_DIRECTORIES
            IMPORTED_LOCATION_DEBUG
            IMPORTED_IMPLIB_DEBUG
            IMPORTED_LOCATION_RELEASE
            IMPORTED_IMPLIB_RELEASE
            IMPORTED_CONFIGURATIONS
            IMPORTED_LOCATION
            IMPORTED_IMPLIB
            )
else()
    message(FATAL_ERROR "Boost is not found! Timing is not disabled! ")
endif()
# #######################################################
# #           Find matlib engine API for C++            #
# #######################################################

# 设置MATLAB根目录
set(MATLAB_ROOT "C:/APP/R2020b" )

# # 查找MATLAB库文件
# find_library(MATLAB_ENGINE_LIB
#     NAMES libMatlabEngine libMatlabEngine.lib
#     PATHS "${MATLAB_ROOT}/extern/lib/win64/microsoft"
#     REQUIRED
# )

# find_library(MATLAB_DATAARRAY_LIB
#     NAMES libMatlabDataArray libMatlabDataArray.lib
#     PATHS "${MATLAB_ROOT}/extern/lib/win64/microsoft"
#     REQUIRED
# )

# # 查找MATLAB头文件
# find_path(MATLAB_INCLUDE_DIR
#     NAMES MatlabDataArray.hpp
#     PATHS "${MATLAB_ROOT}/extern/include"
#     REQUIRED
# )
find_library(MATLAB_ENGINE_LIB
    NAMES libeng libeng.lib
    PATHS "${MATLAB_ROOT}/extern/lib/win64/microsoft"
    REQUIRED
)

find_library(MATLAB_MX_LIB
    NAMES libmx libmx.lib
    PATHS "${MATLAB_ROOT}/extern/lib/win64/microsoft"
    REQUIRED
)

# 查找MATLAB头文件
find_path(MATLAB_INCLUDE_DIR
    NAMES engine.h
    PATHS "${MATLAB_ROOT}/extern/include"
    REQUIRED
)

# 打印查找结果
message(STATUS "MATLAB Engine Library: ${MATLAB_ENGINE_LIB}")
message(STATUS "MATLAB MX Library: ${MATLAB_MX_LIB}")
message(STATUS "MATLAB Include Directory: ${MATLAB_INCLUDE_DIR}")

# #######################################################
# #                     Find CoppeliaSim                #
# #######################################################

set(CoppeliaSim_lib_dir "E:/DJR/VS2019Prebuild/CoppeliaSimLib" )

find_library(coppeliaSim_lib coppeliaSim.dll PATHS  ${CoppeliaSim_lib_dir} REQUIRED )
find_library(coppeliaSim_Head coppeliaSimHeadless.dll PATHS  ${CoppeliaSim_lib_dir} REQUIRED )

set( coppeliaSim_dir "E:/DJR/VS2019Prebuild/CoppeliaSimLib/include" )

# 打印查找结果
message(STATUS "coppeliaSimHeadless: ${CoppeliaSim_Head}")
message(STATUS "CoppeliaSim: ${CoppeliaSim}")


# #######################################################
# #                     Find pinocchio                  #
# #######################################################
# in 404CAD do not have pinocchio
#  if ( 3090_Ubuntu1804 OR t14s_Ubuntu1804 )
#     set( HPP_FCL_PATH /usr/local/lib/cmake/hpp-fcl/ )
#     set( EIGENPY_PATH /usr/local/lib/cmake/eigenpy/ )
# elseif( LiChengNotebook )
#     set( HPP_FCL_PATH /opt/openrobots/lib/cmake/hpp-fcl )
#     set( EIGENPY_PATH /opt/openrobots/lib/cmake/eigenpy )
#     set( PINOCCHIO_PATH /opt/openrobots/lib/cmake/pinocchio )
# endif()

# #   Find pinocchio
# find_package( eigenpy CONFIG REQUIRED PATHS ${EIGENPY_PATH} NO_DEFAULT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_ENVIRONMENT_PATH )
# find_package( hpp-fcl CONFIG REQUIRED PATHS ${HPP_FCL_PATH} )
# find_package( pinocchio CONFIG REQUIRED PATHS ${PINOCCHIO_PATH} )

# if( pinocchio_FOUND )
#     message( STATUS "pinocchio is found!")
# endif()


# #######################################################
# #                     Find Threads                    #
# #######################################################
# find_package(Threads REQUIRED)
# if (Threads_FOUND)
#         message(STATUS "Threads is found!")
# endif()


# #######################################################
# #                     Find Franka                     #
# #######################################################
# if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
#     if ( 3090_Ubuntu1804 OR t14s_Ubuntu1804 )
#         set( Franka_DIR "/usr/local/lib/cmake/Franka" )
#     elseif( LiChengNotebook )
#         set( Franka_DIR "/opt/ros/melodic/share/franka/cmake" )
#     endif()

#     find_package(Franka REQUIRED)
#     if (Franka_FOUND)
#             message(STATUS "Franka is found!")
#     endif()
# endif()

# #######################################################
# #                     Find OSQP                     #
# #######################################################
# set(OSQP_INCLUDE_DIR "E:/DJR/VS2019Prebuild/OSQP_build/include")
# set(OSQP_LIB_DIR "E:/DJR/VS2019Prebuild/OSQP_build/lib")

# # 查找库文件
# include_directories(${OSQP_INCLUDE_DIR})

# # 查找静态库
# find_library(OSQP_LIBRARY
#     osqp  # 库的名称
#     ${OSQP_LIB_DIR}  # 明确指定路径
# )

# # 检查是否找到库
# if (NOT OSQP_LIBRARY)
#     message(FATAL_ERROR "OSQP library not found!")
# else()
#     message("OSQP library is found!")
# endif()
# #######################################################
# #                     Find Qt                         #
# #######################################################


set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package( Qt5 REQUIRED COMPONENTS Core Widgets OpenGL )



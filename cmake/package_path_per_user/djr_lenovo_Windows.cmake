############################################################
#   Find Cutting For Welding dependant packages for user tangtang 4090 Windows

#   The dependant packages include
#   --  Qt 5.15.5
#   --  Boost 1.78.0
#   --  Eigen
#   --  libfranka
#   --  pinocchio
#   Try to find each packages with an imported target
#   Written by djr in 2025.1
############################################################

# #######################################################
# #                     Find Eigen                      #
# #######################################################

find_package(Eigen3 REQUIRED PATHS D:/Eigen340/share/eigen3/cmake )

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
set (Boost_DIR "D:/Boost")  
set(Boost_Components locale date_time filesystem timer regex thread serialization system program_options PATHS ${Boost_DIR} )
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
# #                     Find Qt                         #
# #######################################################


# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTORCC ON)
# find_package(Qt5 COMPONENTS Core Widgets REQUIRED)



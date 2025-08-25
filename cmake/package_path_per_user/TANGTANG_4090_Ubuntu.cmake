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


message( STATUS "Loading 3rd party libraries for 4090 Ubuntu 22.04 System." )
include( CMakePrintHelpers )

#   Temporarily saved module path
set( SAVE_MODULE_PATH ${CMAKE_MODULE_PATH} )
set( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/UbuntuFindPackages ${CMAKE_MODULE_PATH} )

#   1. Find Eigen
set( Eigen_CMAKE_DIR /usr/local/share/eigen3/cmake )
include( UbuntuCMake_FindEigen )

#   2. Find Boost
set( Boost_CMAKE_DIR /usr/local/lib/cmake/Boost-1.78.0 )
include( UbuntuCMake_FindBoost )

#   3. Find Qt5.15.8
set( QT5158_CMAKE_DIR /opt/Qt5.15.8/lib/cmake/Qt5 )
include( UbuntuCMake_FindQt5158 )

#   4. Find pinocchio
set( PINOCCHIO_CMAKE_DIR /usr/local/lib/cmake/pinocchio )
include( UbuntuCMake_FindPinocchio )

#   5. Find Franka
set( Franka_CMAKE_DIR /usr/local/lib/cmake/Franka )
include( UbuntuCMake_FindFranka )


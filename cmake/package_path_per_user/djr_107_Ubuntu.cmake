############################################################
#   Find Cutting For Welding dependant packages for user tangtang 4090 Windows

#   The dependant packages include
#   --  Qt 5.15.5
#   --  Boost 1.78.0
#   --  Eigen
#   --  libfranka
#   --  pinocchio
#   --  torch1.9
#   Try to find each packages with an imported target
#   Written by djr in 2025.1
############################################################

############################################################

# #######################################################
# #                     Find Eigen                      #
# #######################################################

find_package(Eigen3 REQUIRED)

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

set(Boost_Components locale date_time filesystem timer regex thread serialization system program_options)
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

set( coal_PATH /home/djr/ws_djr/djr_libs/hpp-fcl )
#    set( EIGENPY_PATH /opt/openrobots/lib/cmake/eigenpy )
#    set( PINOCCHIO_PATH /opt/openrobots/lib/cmake/pinocchio )
find_package( eigenpy CONFIG REQUIRED )
#find_package( coal CONFIG REQUIRED PATHS ${HPP_FCL_PATH} )
#    find_package( hpp-fcl REQUIRED )
find_package( pinocchio CONFIG REQUIRED )

if( pinocchio_FOUND )
    message( STATUS "pinocchio is found!")
endif()


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

set(Franka_DIR "/home/djr/ws_djr/djr_libs/libfranka/build")
find_package(Franka REQUIRED PATHS ${Franka_DIR} )
if (Franka_FOUND)
    message(STATUS "Franka is found!")
endif()

# #######################################################
# #                     Find Qt                         #
# #######################################################


set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package( Qt5 REQUIRED COMPONENTS Core Widgets OpenGL Charts WebEngineWidgets )
#find_package(Qt5 REQUIRED COMPONENTS Widgets UiTools)
if (Qt5_FOUND)
    message(STATUS "Qt 5 found!")
else()
    message(FATAL_ERROR "Qt 5 not found!")
endif()


## #######################################################
## #                     Find urdfdom                    #
## #######################################################
#find_package( urdfdom CONFIG REQUIRED PATHS /usr/local/lib/urdfdom/cmake NO_DEFAULT_PATH )
#if(urdfdom_FOUND)
#    message(STATUS "urdfdom is FOUND with version = ${urdfdom_Version}!")

#    #urdfdom_sensor;urdfdom_model_state;urdfdom_model;urdfdom_world

#    if( TARGET urdfdom::urdfdom_sensor )
#        message( STATUS "Target urdfdom::urdfdom_sensor is found!")
#    endif()

#    if( TARGET urdfdom::urdf_parser )
#        message( STATUS "Target urdfdom::urdf_parser is found!")
#    else()
#        message( WARNING "Target urdfdom::urdf_parser is NOT found!")
#        add_library(urdf_parser INTERFACE)
##            target_include_directories(urdf_parser INTERFACE
##              "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
##              "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
#        target_link_libraries(urdf_parser INTERFACE
#          urdfdom::urdfdom_model
#          urdfdom::urdfdom_sensor
#          urdfdom::urdfdom_world)
#        add_library(urdfdom::urdf_parser ALIAS urdf_parser)
#    endif()
#else()
#endif()

#message(STATUS "URDFDOM_FOUND: ${URDFDOM_FOUND}")
#message(STATUS "URDFDOM_INCLUDE_DIRS: ${URDFDOM_INCLUDE_DIRS}")
#message(STATUS "URDFDOM_LIBRARIES: ${URDFDOM_LIBRARIES}")

 # #######################################################
 # #                     Find libtorch                   #
 # #######################################################
set(USE_CUDNN OFF)
set(Torch_DIR "/home/djr/djr_workspace/djr_libs/libtorch/share/cmake/Torch")
 find_package( Torch REQUIRED HINTS ${Torch_DIR})

 if(TORCH_FOUND)
     message(STATUS "Found Torch: ${TORCH_INCLUDE_DIRS}")
     message(STATUS "Torch Libraries: ${TORCH_LIBRARIES}")
 else()
     message(FATAL_ERROR "Torch not found!")
 endif()

 # #######################################################
 # #                     Find onnxruntime                #
 # #######################################################
find_package(onnxruntime REQUIRED)


# # #######################################################
# # #                     Find cppzmq                     #
# # #######################################################
# find_package( cppzmq REQUIRED )

# # #######################################################
# # #                     Find jsoncons                   #
# # #######################################################

# find_package( jsoncons REQUIRED )

# #######################################################
# #                     Find ZeroMQ                     #
# #######################################################

find_package( ZeroMQ REQUIRED )


# # #######################################################
# # #            Find libRemoteAPIClient                  #
# # #######################################################

set(RemoteAPIClient "/home/djr/app/CoppeliaSim/programming/zmqRemoteApi/clients/cpp/build" )
find_library(RemoteAPIClient_LIB
    NAMES RemoteAPIClient
    PATHS ${RemoteAPIClient}
    REQUIRED
)

# 查找头文件
set( RemoteAPIClient_INCLUDE_DIR /home/djr/app/CoppeliaSim/programming/zmqRemoteApi/clients/cpp )

# 打印查找结果
message(STATUS "RemoteAPIClient_LIB Library: ${RemoteAPIClient_LIB}")
message(STATUS "RemoteAPIClient_INCLUDE_DIR Include Directory: ${RemoteAPIClient_INCLUDE_DIR}")


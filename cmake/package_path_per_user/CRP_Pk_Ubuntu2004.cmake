############################################################
#   Find Cutting For Welding dependant packages for user tangtang 4090 Windows

#   The dependant packages include
#   --  Qt 5.15.5
#   --  Boost 1.78.0
#   --  Eigen
#   --  libfranka
#   --  pinocchio
#   --  Python3
#   --  nlopt
#   Try to find each packages with an imported target
#   Written by djr in 2025.5
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

find_package( eigenpy CONFIG REQUIRED )
#    find_package( hpp-fcl REQUIRED )
find_package( pinocchio CONFIG REQUIRED )

if( pinocchio_FOUND )
    message( STATUS "pinocchio is found!")
endif()

# #######################################################
# #                     Find Qt                         #
# #######################################################


set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package( Qt5 REQUIRED COMPONENTS Core Widgets OpenGL Charts )
#find_package(Qt5 REQUIRED COMPONENTS Widgets UiTools)
if (Qt5_FOUND)
    message(STATUS "Qt 5 found!")
else()
    message(FATAL_ERROR "Qt 5 not found!")
endif()



# #######################################################
# #                     Find ZeroMQ                     #
# #######################################################

find_package( ZeroMQ REQUIRED )


# # #######################################################
# # #            Find libRemoteAPIClient                  #
# # #######################################################

set(RemoteAPIClient "/home/wasedapk/app/CoppeliaSim/programming/zmqRemoteApi/clients/cpp/build" )
find_library(RemoteAPIClient_LIB
    NAMES RemoteAPIClient
    PATHS ${RemoteAPIClient}
    REQUIRED
)

# 查找头文件
set( RemoteAPIClient_INCLUDE_DIR /home/wasedapk/app/CoppeliaSim/programming/zmqRemoteApi/clients/cpp )

# 打印查找结果
message(STATUS "RemoteAPIClient_LIB Library: ${RemoteAPIClient_LIB}")
message(STATUS "RemoteAPIClient_INCLUDE_DIR Include Directory: ${RemoteAPIClient_INCLUDE_DIR}")

# # #######################################################
# # #                     Find cppzmq                     #
# # #######################################################
 find_package( cppzmq REQUIRED )

# # #######################################################
# # #                     Find jsoncons                   #
# # #######################################################

 find_package( jsoncons REQUIRED )

 # # #######################################################
 # # #                     Find nlopt                      #
 # # #######################################################

 find_package( NLopt REQUIRED )

# # #######################################################
# # #                     Find CRP_SDK                    #
# # #######################################################

set(CRP_SDK "/home/wasedapk/workspace/libs/CrobotpOSSDK" )
set(CRP_SDK_LIB_PATH "/home/wasedapk/workspace/libs/CrobotpOSSDK/bin")
set(CRP_SDK_HEAD_PATH "/home/wasedapk/workspace/libs/CrobotpOSSDK/cpp/include")
find_library(CRP_LIBS
    NAMES RobotService
    PATHS ${CRP_SDK_LIB_PATH}
    REQUIRED
)

# 查找头文件
set( CRP_INCLUDE_DIR ${CRP_SDK_HEAD_PATH} )

# 打印查找结果
message(STATUS "CRP_LIB Library: ${CRP_LIBS}")
message(STATUS "CRP_HEAD Include Directory: ${CRP_INCLUDE_DIR}")

# # #######################################################
# # #                     Find spdlog                    #
# # #######################################################
find_package( spdlog REQUIRED)

# # #######################################################
# # #            Find HumanoidDualArmSolver               #
# # #######################################################
set(HuamniodRobot "/home/wasedapk/workspace/libs/HumanoidDualArmSolver" )
set(HuamniodRobot_include "${HuamniodRobot}/include")

find_library(MYLIBTI5_LIB
    NAMES mylibti5_2004
    PATHS "${HuamniodRobot}/usrlib/2004"
    REQUIRED
)
find_library(CONTROLCAN_LIB
    NAMES controlcan
    PATHS "${HuamniodRobot}/usrlib/2004"
    REQUIRED
)

set(HuamniodRobot_LIBS
    ${MYLIBTI5_LIB}
    ${CONTROLCAN_LIB}
)

# 打印查找结果
message(STATUS "HuamniodRobot_include: ${HuamniodRobot_include}")
message(STATUS "MYLIBTI5_LIB Library: ${MYLIBTI5_LIB}")
message(STATUS "CONTROLCAN_LIB Library: ${CONTROLCAN_LIB}")
message(STATUS "HuamniodRobot_LIBS Library: ${HuamniodRobot_LIBS}")

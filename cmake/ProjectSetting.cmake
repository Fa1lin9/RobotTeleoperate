############################################################
#   Project Setting
############################################################
message( STATUS "Operation System is : ${CMAKE_SYSTEM} ${CMAKE_SIZEOF_VOID_P}" )

############################################################
#   Set the C++ standard
#   C++ standard 17 is required for Win10 Visual Studio 2019
############################################################
if ( CMAKE_SYSTEM_NAME MATCHES "Linux" )
    message(STATUS "current platform: Linux " )
    set( CMAKE_C_COMPILER "/usr/bin/gcc" )
    set( CMAKE_CXX_COMPILER "/usr/bin/g++" )
    set( CMAKE_CXX_STANDARD 17 )

elseif( CMAKE_SYSTEM_NAME MATCHES "Windows" )
    message( STATUS "current platform: Windows" )
    set( CMAKE_CXX_STANDARD 17 )
else ()
    message( STATUS "other platform: ${CMAKE_SYSTEM_NAME}" )
endif ()

############################################################
#   Check the system bits (Correct Way)
############################################################
message( STATUS "CMAKE_SIZEOF_VOID_P is ${CMAKE_SIZEOF_VOID_P}." )
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    message( STATUS "Target is 64 bits" )
    set( CMAKE_DEBUG_POSTFIX        "_dx64" )
    set( CMAKE_RELEASE_POSTFIX      "_rx64" )
else()
    message( STATUS "Target is 32 bits" )
    set( CMAKE_DEBUG_POSTFIX        "_dx86" )
    set( CMAKE_RELEASE_POSTFIX      "_rx86" )
endif()


############################################################
#   Set the output position
############################################################
#   Output directory setting
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/$<CONFIG>/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/$<CONFIG>/lib ) 
#	Windows library output
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/$<CONFIG>/lib )

#   Install prefix setting
set( CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/PackagesInstallation/ )


############################################################
#   Build options setting
############################################################
#	Set configuration : Debug, Release
set( CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configurations" FORCE )
set( CMAKE_BUILD_TYPE "Debug" CACHE STRING "Current Configuration" )

#   Use folders for different target in Visual Studio
set_property( GLOBAL PROPERTY USE_FOLDERS ON )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
# set( CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON )
set( CMAKE_POSITION_INDEPENDENT_CODE ON )

if( CMAKE_VERSION VERSION_LESS "3.7.0")
    set( CMAKE_INCLUDE_CURRENT_DIR ON )
endif()

# SET( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj" )


############################################################
#   Find Eigen in Ubuntu 22.04
#   Written by Tang Qing in Mar. 2024.
############################################################

#   Show the current CMake file
get_filename_component( this_cmake_file ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE )
message( STATUS "\n-------------------- Finding Eigen in ${this_cmake_file} --------------------------" )


#   Define the Eigen package CMake directory.
if( NOT DEFINED Eigen_CMAKE_DIR )
    get_filename_component( Eigen_CMAKE_DIR /usr/local/share/eigen3/cmake ABSOLUTE )
else()
    message( STATUS "Using custom defined package directory!" )
endif()
message( STATUS "Eigen_CMAKE_DIR is : ${Eigen_CMAKE_DIR}." )


#   Find package
find_package( Eigen3 CONFIG PATHS ${Eigen_CMAKE_DIR} )
if( Eigen3_FOUND )
    if( TARGET Eigen3::Eigen )
        message( STATUS "Target Eigen3::Eigen version = ${Eigen3_VERSION} is FOUND!" )
        cmake_print_properties( TARGETS Eigen3::Eigen PROPERTIES
            INCLUDE_DIRECTORIES
            INTERFACE_INCLUDE_DIRECTORIES
            IMPORTED_LOCATION_DEBUG
            IMPORTED_IMPLIB_DEBUG
            IMPORTED_LOCATION_RELEASE
            IMPORTED_IMPLIB_RELEASE
            IMPORTED_CONFIGURATIONS
            )
        
            #   How to Using Eigen
        message( STATUS "\nTips of Using Eigen:" )
        message( STATUS "-- Using Eigen libs: Eigen3::Eigen. " )
    else()
        message( STATUS "Target Eigen3::Eigen is NOT FOUND!" )
    endif()
else()
    message( STATUS "Eigen3 version ${Eigen3_VERSION} is NOT FOUND!" )
endif()

  
#   Show the current CMake file
message( STATUS "-------------------- Finding Eigen ${Eigen3_VERSION} in ${this_cmake_file} --------------------\n" )



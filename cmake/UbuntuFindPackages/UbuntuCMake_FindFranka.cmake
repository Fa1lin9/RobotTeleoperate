#############################################################
#	Find Franka version 3.3.1
#   Written by Tang Qing in Jan 2025.
#############################################################

#   Show the current CMake file
get_filename_component( this_cmake_file ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE )
message(STATUS "\n---------- Finding Franka in ${this_cmake_file} ----------")


find_package( Franka PATHS ${Franka_CMAKE_DIR} )

if( Franka_FOUND )
    message( STATUS "Franka version ${Franka_VERSION} is found!" )

    if( TARGET Franka::Franka )
        message( STATUS "Target - Franka::Franka is FOUND!" )

        #   Print target information
        if( ${Display_Imported_Target} )
        cmake_print_properties( TARGETS Franka::Franka
            PROPERTIES
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
    else()
        message( STATUS "Target - Franka::Franka is NOT found!" )
    endif()

    #   How to use Franka
    message( STATUS "\nTips of Using Franka ${Franka_VERSION}: " )
    message( STATUS "-- Using Franka targets: Franka::Franka" )
else()
    message( FATAL_ERROR "Franka is NOT found!" )
endif()


#   Show the current CMake file
message( STATUS "---------- Finding Franka version ${Franka_VERSION} in ${this_cmake_file} ----------\n" )


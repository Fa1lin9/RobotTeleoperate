#############################################################
#	Find Pinocchio version 3.3.1
#   Written by Tang Qing in Jan 2025.
#############################################################

#   Show the current CMake file
get_filename_component( this_cmake_file ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE )
message(STATUS "\n---------- Finding pinocchio in ${this_cmake_file} ----------")


# set( QT_Components 
#     Core
#     Gui
#     Widgets
#     OpenGL
#     Charts
# )

find_package( pinocchio PATHS ${PINOCCHIO_CMAKE_DIR} )

if( pinocchio_FOUND )
    message( STATUS "pinocchio version ${pinocchio_VERSION} is found!" )

    if( TARGET pinocchio::pinocchio )
        message( STATUS "Target - pinocchio::pinocchio is FOUND!" )

        #   Print target information
        if( ${Display_Imported_Target} )
        cmake_print_properties( TARGETS pinocchio::pinocchio
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
        message( STATUS "Target - pinocchio::pinocchio is NOT found!" )
    endif()

    #   How to use pinocchio
    message( STATUS "\nTips of Using pinocchio ${pinocchio_VERSION}: " )
    message( STATUS "-- Using pinocchio targets: pinocchio::pinocchio" )
else()
    message( FATAL_ERROR "pinocchio is NOT found!" )
endif()


#   Show the current CMake file
message( STATUS "---------- Finding pinocchio version ${pinocchio_VERSION} in ${this_cmake_file} ----------\n" )


#############################################################
#	Find Qt version 5.15.8
#   Written by Tang Qing in Jan 2025.
#############################################################

#   Show the current CMake file
get_filename_component( this_cmake_file ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE )
message(STATUS "\n---------- Finding Qt5 in ${this_cmake_file} ----------")


set( QT_Components 
    Core
    Gui
    Widgets
    OpenGL
    Charts
)

find_package( Qt5 REQUIRED COMPONENTS ${QT_Components} PATHS ${QT5158_CMAKE_DIR} )

if( Qt5_FOUND )
    message( STATUS "Qt5 version ${Qt5_VERSION} is found!" )

    foreach( each_component ${QT_Components} )
        if( TARGET Qt5::${each_component} )
            message( STATUS "Target - Qt5::${each_component} is FOUND!" )
            set( Qt5_Found_Components ${Qt5_Found_Components} Qt5::${each_component} )

            #   Print target information
            if( ${Display_Imported_Target} )
            cmake_print_properties( TARGETS Qt5::${each_component}
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
            message( STATUS "Target - Qt5::${each_component} is NOT found!" )
        endif()
    endforeach()

    #   How to use Qt5
    message( STATUS "\nTips of Using Qt5 ${Qt5_VERSION}: " )
    message( STATUS "-- Using Qt5 targets: ${Qt5_Found_Components}" )
else()
    message( FATAL_ERROR "Qt5 is NOT found!" )
endif()


#   Show the current CMake file
message(STATUS "---------- Finding Qt5 version ${Qt5_VERSION} in ${this_cmake_file} ----------\n")


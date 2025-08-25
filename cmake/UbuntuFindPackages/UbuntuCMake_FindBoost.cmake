############################################################
#   Find Eigen in Ubuntu 22.04
#   Written by Tang Qing in Mar. 2024.
############################################################

#   Show the current CMake file
get_filename_component( this_cmake_file ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE )
message( STATUS "\n-------------------- Finding Eigen in ${this_cmake_file} --------------------------" )


#   Define the Eigen package CMake directory.
if( NOT DEFINED Boost_CMAKE_DIR )
    get_filename_component( Boost_CMAKE_DIR /usr/local/lib/cmake/Boost-1.76.0 ABSOLUTE )
else()
    message( STATUS "Using custom defined package directory!" )
endif()
message( STATUS "Boost_CMAKE_DIR is : ${Boost_CMAKE_DIR}." )



#   Find boost setting
set( Boost_USE_STATIC_LIBS           	OFF )    # only find static libs
#   如果需要构建Debug程序，下述选项应该勾选ON
set( Boost_USE_DEBUG_LIBS            	ON )	# ignore debug libs and
set( Boost_USE_RELEASE_LIBS          	ON )    # only find release libs
set( Boost_USE_MULTITHREADED        	ON )
set( Boost_USE_STATIC_RUNTIME        	OFF )

#   Set the components and find the Boost package
set( Boost_Components locale date_time filesystem timer regex thread serialization system program_options python random exception log log_setup json )

#   Find package
find_package( Boost REQUIRED COMPONENTS ${Boost_Components} PATHS ${Boost_CMAKE_DIR} )
if( Boost_FOUND )
    message( STATUS "Found Boost ${Boost_VERSION} at ${Boost_CMAKE_DIR}!" )

    foreach( each_component  ${Boost_Components} )
        if( TARGET Boost::${each_component} )
            # message( STATUS "Boost Component Imported Target Boost::${each_component} is found!" )
            set( Boost_Found_Components ${Boost_Found_Components} Boost::${each_component} )
        else()
            message( STATUS "Boost Component Imported Target Boost::${each_component} is NOT found!" )
        endif()
    endforeach()

    #   How to Using Boost
    message( STATUS "\nTips of Using Boost:" )
    message( STATUS "-- Using Boost headers: Boost::headers" )
    message( STATUS "-- Using Boost libs: ${Boost_Found_Components}. " )
else()
    message( STATUS "Boost is NOT found!" )
endif( Boost_FOUND )

#   Tips of Using Boost
#   Boost::date_time, Boost::filesystem

  
#   Show the current CMake file
message( STATUS "-------------------- Finding Boost ${Boost_VERSION} in ${this_cmake_file} --------------------\n" )



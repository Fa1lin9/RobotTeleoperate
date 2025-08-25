############################################################
#   Find Cutting For Welding dependant packages for user tangtang 4090 Windows

#   The dependant packages include
#   --  Qt 5.15.5
#   --  Boost 1.78.0
#   --  ortools 9.7
#   --  OpenCV 3.4.18
#   --  Eigen
#   --  OpenMP
#   Try to find each packages with an imported target
#   Written by Tang Qing in Oct. 2023.
############################################################

#   Define the message header
set( message_header "[Find packs for 4090_Win] :" )

#   Save the original CMake Module Path
set( SAVE_MODULE_PATH ${CMAKE_MODULE_PATH} )

# #   Set the prebuild path
# set( vsindep_DIR  E:/Prebuild/vs_indep/cmake CACHE FILEPATH "file path" FORCE )
# set( vs2019_DIR  E:/Prebuild/vs2019/cmake CACHE FILEPATH "file path" FORCE )


#   Set the prebuild directory
set( Prebuild_DIR "D:/PreBuild" )
message( STATUS "${message_header} The prebuild directory is ${Prebuild_DIR}" )

#   Find the packages built with visual studio 2019
get_filename_component( WIN_VS2019_DIR ${Prebuild_DIR}/vs2019/cmake ABSOLUTE )
set( CMAKE_MODULE_PATH ${WIN_VS2019_DIR} )

# #   Find OpenCV
# include( CMake_FindOpenCV3 )

#   Start finding Boost
set( Boost_USE_STATIC_LIBS           	OFF )    # only find static libs
# 如果需要构建Debug程序，下述选项应该勾选ON
set( Boost_USE_DEBUG_LIBS            	ON )	# ignore debug libs and
set( Boost_USE_RELEASE_LIBS          	ON )    # only find release libs
set( Boost_USE_MULTITHREADED        	ON )
set( Boost_USE_STATIC_RUNTIME        	OFF )
# include( CMake_FindBoost176 )
include( CMake_FindBoost178py311 )

#   Find or-tools
# include( CMake_FindORTools96rd )

#   Find google test
# include( CMake_FindGoogleTestStatic )
# include( CMake_FindGoogleTestShared )


#   Find pinnocchio
include( CMake_Findpinocchio )




#   Find the packages independent of visual studio
get_filename_component( WIN_VSIndep_DIR ${Prebuild_DIR}/vs_indep/cmake ABSOLUTE )
set( CMAKE_MODULE_PATH ${WIN_VSIndep_DIR} )
#   Find Qt

option( Using_Qt5 "Whether use Qt5/Qt6." ON )
if( Using_Qt5 )
    include( CMake_FindQt5155 )
else()
    include( CMake_FindQt624 )
endif()

#   Find Eigen
include( CMake_FindEigen )


# #   Whether use OpenMP
# set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/Ubuntu )
# include( FindOpenMP )


set( CMAKE_MODULE_PATH ${SAVE_MODULE_PATH} )




message( STATUS "${message_header} End of finding dependant packages for the projects." )
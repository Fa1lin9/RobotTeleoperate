############################################################
#   Define three install functions for RobotSimulator 2020

#   The install functions include
#   --  install_target
#       The INCLUDES directory for cmake only includes "include"
#   --  install_target_dir
#       The INCLUDES directory for cmake includes "include" and "include/${TARGET_NAME}"
#   --  install_target_customized_dir
#       Only install the header files for the project. Each header files is in a special directory.
#       Call the function for each directory and specify the special directory in each function.

#   Try to install each packages to a cmake file
#   Written by Tang Qing in Dec. 2020.
############################################################


include(CMakePackageConfigHelpers)

function( test_func arg1 arg2 arg3 )
    message( STATUS "arg1 is : ${arg1}." )
    message( STATUS "arg2 is : ${arg2}." )
    message( STATUS "arg3 is : ${arg3}." )

    message( STATUS "argc is ${ARGC}." )
    message( STATUS "argv is ${ARGV}." )
    message( STATUS "argn is ${ARGN}." )
endfunction( test_func )



#	Define an installation function
function( install_target _target_name _group_name _headers )

    message( "${BoldYellow}-- Install ${_group_name}::${_target_name} ${ColourReset}" )

    #   Install the target libs
    install( TARGETS ${_target_name}
        EXPORT ${_target_name}_export_debug
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${_group_name}/bin
        LIBRARY DESTINATION ${_group_name}/lib
        ARCHIVE DESTINATION ${_group_name}/lib
        INCLUDES DESTINATION ${_group_name}/include
    )

    install( TARGETS ${_target_name}
        EXPORT ${_target_name}_export_release
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${_group_name}/bin
        LIBRARY DESTINATION ${_group_name}/lib
        ARCHIVE DESTINATION ${_group_name}/lib
        INCLUDES DESTINATION ${_group_name}/include
    )

    #   Export the "Targets.cmake"
    install(
        EXPORT ${_target_name}_export_debug
        CONFIGURATIONS Debug
        FILE ${_target_name}Targets.cmake
        NAMESPACE "${_group_name}::"
        DESTINATION ${_group_name}/lib/cmake/${_target_name}
    )

    install(
        EXPORT ${_target_name}_export_release
        CONFIGURATIONS Release
        FILE ${_target_name}Targets.cmake
        NAMESPACE "${_group_name}::"
        DESTINATION ${_group_name}/lib/cmake/${_target_name}
    )

    #   To export pdb files for shared library
    #   Only needed in Windows system
    if( CMAKE_SYSTEM_NAME MATCHES "Linux")
        #   Needed only in Windows System
    elseif ( CMAKE_SYSTEM_NAME MATCHES "Windows")
        get_target_property( target_type ${_target_name} TYPE )
        if ( target_type STREQUAL SHARED_LIBRARY )
            install( FILES $<TARGET_PDB_FILE:${_target_name}> DESTINATION ${_group_name}/bin OPTIONAL )
        endif()
    endif ()
    # $<BUILD_INTERFACE:$<$<STREQUAL:${CMAKE_SYSTEM_NAME},Linux>:dl>>


    #   Install the Head files
    if( "${_headers}" STREQUAL "" )
    else()
        message( STATUS "${message_header} ARGC = ${ARGC}" )
        message( STATUS "${message_header} ARGV = ${ARGV}" )

        set(INDEX 3)
        
        while(INDEX LESS ${ARGC})
            # message("ARG = " ${ARGV${INDEX}})
            list( APPEND _headers ${ARGV${INDEX}} )
            math(EXPR INDEX "${INDEX} + 1")
        endwhile()

        message(STATUS "${message_header} ${_headers}")
        
        foreach( _head_file ${_headers} )
            install( FILES ${_head_file} DESTINATION ${_group_name}/include/${_target_name} )
        endforeach()
    endif()

endfunction()

#	Define an installation function
function( install_target_binaryonly _target_name _group_name )

    message( "${BoldYellow}-- Install ${_group_name}::${_target_name} ${ColourReset}" )

    #   Install the target libs
    install( TARGETS ${_target_name}
        EXPORT ${_target_name}_export_debug
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${_group_name}/bin
        LIBRARY DESTINATION ${_group_name}/lib
        ARCHIVE DESTINATION ${_group_name}/lib
        INCLUDES DESTINATION ${_group_name}/include
    )

    install( TARGETS ${_target_name}
        EXPORT ${_target_name}_export_release
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${_group_name}/bin
        LIBRARY DESTINATION ${_group_name}/lib
        ARCHIVE DESTINATION ${_group_name}/lib
        INCLUDES DESTINATION ${_group_name}/include
    )

    #   Export the "Targets.cmake"
    install(
        EXPORT ${_target_name}_export_debug
        CONFIGURATIONS Debug
        FILE ${_target_name}Targets.cmake
        NAMESPACE "${_group_name}::"
        DESTINATION ${_group_name}/lib/cmake/${_target_name}
    )

    install(
        EXPORT ${_target_name}_export_release
        CONFIGURATIONS Release
        FILE ${_target_name}Targets.cmake
        NAMESPACE "${_group_name}::"
        DESTINATION ${_group_name}/lib/cmake/${_target_name}
    )

    #   To export pdb files for shared library
    #   Only needed in Windows system
    if( CMAKE_SYSTEM_NAME MATCHES "Linux")
        #   Needed only in Windows System
    elseif ( CMAKE_SYSTEM_NAME MATCHES "Windows")
        get_target_property( target_type ${_target_name} TYPE )
        if ( target_type STREQUAL SHARED_LIBRARY )
            install( FILES $<TARGET_PDB_FILE:${_target_name}> DESTINATION ${_group_name}/bin OPTIONAL )
        endif()
    endif ()
    # $<BUILD_INTERFACE:$<$<STREQUAL:${CMAKE_SYSTEM_NAME},Linux>:dl>>


endfunction()


# function( install_target _target_name _group_name _headers )
#     #   Install the target libs
#     message( "${BoldYellow}-- Install ${_group_name}::${_target_name} ${ColourReset}" )
#     install( TARGETS ${_target_name}
#         EXPORT ${_target_name}Targets
#         RUNTIME DESTINATION ${_group_name}/bin
#         LIBRARY DESTINATION ${_group_name}/lib
#         ARCHIVE DESTINATION ${_group_name}/lib
#         INCLUDES DESTINATION ${_group_name}/include
#     )

#     install(
#         EXPORT ${_target_name}Targets
#         FILE ${_target_name}Targets.cmake
#         NAMESPACE ${_group_name}::
#         DESTINATION ${_group_name}/lib/cmake/${_target_name}
#     )
# endfunction()


#	Define an installation function
function( install_target_dir _target_name _group_name _headers )

    #   Install the target libs
    install( TARGETS ${_target_name}
        CONFIGURATIONS Debug
        EXPORT ${_target_name}_export_debug
        RUNTIME DESTINATION ${_group_name}/bin
        LIBRARY DESTINATION ${_group_name}/lib
        ARCHIVE DESTINATION ${_group_name}/lib
        INCLUDES DESTINATION ${_group_name}/include ${_group_name}/include/${_target_name}
    )

    install( TARGETS ${_target_name}
        CONFIGURATIONS Release
        EXPORT ${_target_name}_export_release
        RUNTIME DESTINATION ${_group_name}/bin
        LIBRARY DESTINATION ${_group_name}/lib
        ARCHIVE DESTINATION ${_group_name}/lib
        INCLUDES DESTINATION ${_group_name}/include ${_group_name}/include/${_target_name}
    )

    #   Export the "Targets.cmake"
    install(
        EXPORT ${_target_name}_export_debug
        CONFIGURATIONS Debug
        FILE ${_target_name}Targets.cmake
        NAMESPACE "${_group_name}::"
        DESTINATION ${_group_name}/lib/cmake/${_target_name}
        )

    install(
        EXPORT ${_target_name}_export_release
        CONFIGURATIONS Release
        FILE ${_target_name}Targets.cmake
        NAMESPACE "${_group_name}::"
        DESTINATION ${_group_name}/lib/cmake/${_target_name}
        )


    #   Install the Head files
    if( ${_headers} STREQUAL "" )
    else()
        message( STATUS "ARGC = ${ARGC}" )
        message( STATUS "ARGV = ${ARGV}" )

        set(INDEX 3)
        
        while(INDEX LESS ${ARGC})
            # message("ARG = " ${ARGV${INDEX}})
            list( APPEND _headers ${ARGV${INDEX}} )
            math(EXPR INDEX "${INDEX} + 1")
        endwhile()

        message(STATUS "${_headers}")
        
        foreach( _head_file ${_headers} )
            install( FILES ${_head_file} DESTINATION ${_group_name}/include/${_target_name} )
        endforeach()
    endif()

endfunction()


#	Define an function which only install the head files
#   Especially when there are different directories for different header files
function( install_target_customized_dir _target_name _group_name _customized_dir _headers )
    #   Install the Head files
    if( ${_headers} STREQUAL "" )
    else()
        message( STATUS "ARGC = ${ARGC}" )
        message( STATUS "ARGV = ${ARGV}" )

        set(INDEX 3)
        
        while(INDEX LESS ${ARGC})
            # message("ARG = " ${ARGV${INDEX}})
            list( APPEND _headers ${ARGV${INDEX}} )
            math(EXPR INDEX "${INDEX} + 1")
        endwhile()

        message(STATUS "${_headers}")
        
        foreach( _head_file ${_headers} )
            install( FILES ${_head_file} DESTINATION ${_group_name}/include/${_target_name}/${_customized_dir} )
        endforeach()
    endif()

endfunction()




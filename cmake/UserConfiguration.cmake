#######################################################
#   User Configuration
#   Make sure only one user is defined in the program
#######################################################

#   Choose User
option( USER_TANGTANG_p15v3_Ubuntu "Configure tangtang's p15v3 notebook with Windows system." OFF )
option( USER_TANGTANG_p15v3_Windows "Configure tangtang's p15v3 notebook with Windows system." OFF )
option( USER_TANGTANG_t14s_Ubuntu "Configure tangtang's t14s notebook with Ubuntu system." OFF )
option( USER_TANGTANG_t14s_Windows "Configure tangtang's t14s notebook with Windows system." OFF )

option( USER_TANGTANG_3090_Ubuntu "Configure tangtang's 3090 desktop with Ubuntu system." OFF )
option( USER_TANGTANG_3090_Windows "Configure tangtang's 3090 desktop with Windows system." OFF )
option( USER_TANGTANG_4090_Ubuntu "Configure tangtang's 4090 desktop with Windows system." OFF )
option( USER_TANGTANG_4090_Windows "Configure tangtang's 4090 desktop with Windows system." OFF )

option( USER_djr_CAD_404_Windows "Configure djr's 404 desktop with Windows system." OFF )
option( USER_djr_lenovo_Windows "Configure djr's lenovo notebook with Windows system." OFF )
option( USER_djr_lenovo_Ubuntu "Configure djr's lenovo notebook with Ubuntu system." OFF )
option( USER_djr_107_Ubuntu "Configure djr's 107 desktop with Ubuntu system." OFF )
option( USER_djr_107_Ubuntu_away_from_door "Configure djr's 107 desktop away from door with Ubuntu system." OFF )

option( USER_CRP_Pk_Ubuntu2004 "Configure crp pk Ubuntu2004 system." OFF )
option( USER_CHF_Ubuntu "Configure CHF Ubuntu system." ON )

#   Make Users as a group
list( APPEND USER_GROUP
    USER_TANGTANG_p15v3_Ubuntu
    USER_TANGTANG_p15v3_Windows
    USER_TANGTANG_t14s_Ubuntu
    USER_TANGTANG_t14s_Windows

    USER_TANGTANG_3090_Ubuntu
    USER_TANGTANG_3090_Windows
    USER_TANGTANG_4090_Ubuntu
    USER_TANGTANG_4090_Windows
    USER_djr_CAD_404_Windows
    USER_djr_lenovo_Windows
    USER_djr_lenovo_Ubuntu
    USER_djr_107_Ubuntu
    USER_djr_107_Ubuntu_away_from_door

    USER_CRP_Pk_Ubuntu2004
    USER_CHF_Ubuntu
)

#   Get how many users are selected
set( NUMBER_OF_USER 0 )
foreach( each_user ${USER_GROUP} )
    if( ${each_user} )
        math( EXPR NUMBER_OF_USER "${NUMBER_OF_USER} + 1" )
        message( STATUS "Current User is ${each_user}. " )
    endif()
endforeach()

#   Return a fatal error if the selected user is not equal to 1
if( NOT ${NUMBER_OF_USER} EQUAL 1 )
    message( FATAL_ERROR "The number of user defined is ${NUMBER_OF_USER}, not equal to 1." )
endif()



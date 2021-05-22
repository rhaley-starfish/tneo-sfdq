
# Display list of objects with message
function(display_list MSGTYPE dispObjs msg)
    if(dispObjs)
        foreach(dispObj ${dispObjs})
            message(${MSGTYPE} "${msg}:  ${dispObj}")
        endforeach()
    endif()
endfunction()

# Display list of displayType (e.g. INCLUDE_DIRECTORIES) objects for target, targ
function(display_target MSGTYPE targ displayType)
    get_target_property(dispObjs ${targ} ${displayType})
    if(dispObjs)
        foreach(dispObj ${dispObjs})
            message(${MSGTYPE} "${targ} ${displayType}:  ${dispObj}")
        endforeach()
    endif()
endfunction()

# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(cartographerTaskLib)
ADD_LIBRARY(${CARTOGRAPHER_TASKLIB_NAME} SHARED 
    ${CARTOGRAPHER_TASKLIB_SOURCES})
add_dependencies(${CARTOGRAPHER_TASKLIB_NAME}
    regen-typekit)

TARGET_LINK_LIBRARIES(${CARTOGRAPHER_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${CARTOGRAPHER_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${CARTOGRAPHER_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${CARTOGRAPHER_TASKLIB_INTERFACE_LIBRARIES}")
SET_TARGET_PROPERTIES(${CARTOGRAPHER_TASKLIB_NAME}
    PROPERTIES INTERFACE_LINK_LIBRARIES "${CARTOGRAPHER_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${CARTOGRAPHER_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${CARTOGRAPHER_TASKLIB_HEADERS}
    DESTINATION include/orocos/cartographer)


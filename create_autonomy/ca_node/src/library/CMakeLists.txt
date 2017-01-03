##############################################################################
# SOURCES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# LIBRARY
##############################################################################

add_library(ca_driver ${SOURCES})
add_dependencies(ca_driver ca_msgs_gencpp)
target_link_libraries(ca_driver
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

install(TARGETS ca_driver
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


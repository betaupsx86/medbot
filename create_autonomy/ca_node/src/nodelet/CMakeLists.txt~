##############################################################################
# NODELET
##############################################################################

add_library(ca_nodelet ca_nodelet.cpp)
add_dependencies(ca_nodelet ca_driver)
target_link_libraries(ca_nodelet ca_driver)

install(TARGETS ca_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

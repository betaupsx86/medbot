cmake_minimum_required(VERSION 2.8.3)
project(breaker_board_node)

find_package(catkin REQUIRED COMPONENTS message_generation
					std_msgs
                                        )
                     
add_service_files(
			DIRECTORY srv
                  	FILES SetBreakerOutputs.srv
)


generate_messages(DEPENDENCIES std_msgs
#				diagnostic_msgs
#                              geometry_msgs
#                              nav_msgs

)
                               

catkin_package(CATKIN_DEPENDS message_runtime
				std_msgs                             
 )

#install(PROGRAMS scripts/kinect_breaker_enabler.py
#                 nodes/breaker_board_node.py
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


catkin_install_python(PROGRAMS 
	nodes/breaker_board.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)

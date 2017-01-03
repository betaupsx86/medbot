cmake_minimum_required(VERSION 2.8.3)
project(create_2_dashboard)

find_package(catkin REQUIRED COMPONENTS ca_node
                                        diagnostic_msgs
                                        rospy
                                        rqt_gui
                                        rqt_gui_py
                                        rqt_robot_dashboard)

catkin_python_setup()

catkin_package()

install(PROGRAMS scripts/create_2_dashboard
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project( roboy_rqt_plugins )

add_subdirectory( pabiroboy_joint_control )
add_subdirectory( roboy_darkroom )
add_subdirectory( roboy_motor_status )
add_subdirectory( roboy_motor_command )

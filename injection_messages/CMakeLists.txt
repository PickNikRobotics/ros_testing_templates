cmake_minimum_required(VERSION 3.0.2)
project(injection_messages)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_service_files(DIRECTORY srv FILES OneNumber.srv)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)

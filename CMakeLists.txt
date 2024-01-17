cmake_minimum_required(VERSION 3.10)
project(FrankaExamples VERSION 0.1 LANGUAGES CXX)

option(BUILD_PYTHON_MODULE "Build python module" ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Franka REQUIRED)

message("Found Eigen Version: ${Eigen3_VERSION}")
message("Found Threads Version: ${Threads_VERSION}")
message("Found Franka Version: ${Franka_VERSION}")

add_library(frankactrl STATIC
	src/utils.cpp
	src/robot.cpp
	src/gripper.cpp
	src/lssab_trajectory.cpp
	src/ssab_trajectory.cpp
)
target_include_directories(frankactrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(frankactrl PUBLIC Franka::Franka Eigen3::Eigen)

if(BUILD_PYTHON_MODULE)
	# Check if pybind11 exists as a subdirectory
	if(EXISTS pybind11)
		add_subdirectory(pybind11)
	else()
		find_package(Python3 COMPONENTS Interpreter Development)
		find_package(pybind11 2.6 REQUIRED)
	endif()
				
	pybind11_add_module(_pyfranka src/python.cpp)
	target_include_directories(_pyfranka PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
	target_link_libraries(_pyfranka PRIVATE frankactrl)
	
#	# Copy python module to package directory
#	add_custom_command(TARGET _pyfranka POST_BUILD
#    	COMMAND ${CMAKE_COMMAND} -E copy_if_different
#        $<TARGET_FILE:_pyfranka>
#        ${CMAKE_CURRENT_SOURCE_DIR}/pyfranka
#    )

endif()

set(EXAMPLES
	franka_move_to_relative_pose
	franka_move_to_relative_joints
	franka_move_velocity
	franka_move_gripper
)

foreach(example ${EXAMPLES})
	add_executable(${example} ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}.cpp)
	target_include_directories(${example} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
	target_link_libraries(${example} PRIVATE frankactrl Franka::Franka Eigen3::Eigen)
endforeach()

install(TARGETS frankactrl
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

cmake_minimum_required(VERSION 3.20)
project(pheV3)

# Set policy CMP0072 to prefer GLVND if available
cmake_policy(SET CMP0072 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE PHEV3_PHYSICS_SRC src/physics/*.cpp)
file(GLOB_RECURSE PHEV3_GRAPHICS_SRC src/graphics/*.cpp)

# Create the main library target
add_library(pheV3 STATIC
		${PHEV3_PHYSICS_SRC}
)

# Set PUBLIC include directory for the library
target_include_directories(pheV3
		PUBLIC ${PROJECT_SOURCE_DIR}/include
		PRIVATE ${PROJECT_SOURCE_DIR}/src
)

# Only try building tests if we are the outer project
if (PROJECT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)

	message(STATUS "Building PheV3 examples")

	# Find packages and external libraries
	find_package(OpenGL REQUIRED)
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(GLFW REQUIRED glfw3)
	pkg_check_modules(GLEW REQUIRED glew)

	add_library(pheV3_graphics STATIC
		${PHEV3_GRAPHICS_SRC}
	)

	target_link_libraries(pheV3_graphics PUBLIC
			pheV3
			OpenGL::GL
			${GLFW_LIBRARIES}
			${GLEW_LIBRARIES}
	)

	add_executable(singleDemo tests/single.cpp)
	target_link_libraries(singleDemo PRIVATE pheV3_graphics)

	add_executable(collisionDemo tests/collision.cpp)
	target_link_libraries(collisionDemo PRIVATE pheV3_graphics)

	add_executable(stackingDemo tests/stacking.cpp)
	target_link_libraries(stackingDemo PRIVATE pheV3_graphics)

	add_executable(headlessDemo tests/headless.cpp)
	target_link_libraries(headlessDemo PRIVATE pheV3)

else()
	message(STATUS "PheV3 is not the root project, tests skipped")
endif()


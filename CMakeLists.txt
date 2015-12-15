cmake_minimum_required(VERSION 2.8)
project(matchmover) 

#@bluking
#add parameters for project files
set(MATCHMOVER_INCLUDE_DIR
	src/include
)
set(MATCHMOVER_SOURCE_DIR
	src/sources
)
set(MATCHMOVER_INCLUDE_FILES
	src/include/arcball.h
	src/include/calibration.h
	src/include/camera.h
	src/include/multiview.h
	src/include/params.h
	src/include/rendering.h
	src/include/stereoview.h
	src/include/teapot.h
	src/include/utils.h
	src/include/videodevice.h
	src/include/videoplayer.h
)
set(MATCHMOVER_SOURCES_FILES
	src/sources/arcball.cpp
	src/sources/calibration.cpp
	src/sources/camera.cpp
	src/sources/main.cpp
	src/sources/multiview.cpp
	src/sources/params.cpp
	src/sources/rendering.cpp	
	src/sources/stereroview.cpp
	src/sources/videodevice.cpp
	src/sources/videoplayer.cpp
)

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

#@bluking
#add package
find_package(OpenCV REQUIRED) 
find_package(SDL REQUIRED)
find_package(PkgConfig)
pkg_check_modules(GTKMM gtkmm-2.4)


#@bluking
#add include dirs
include_directories(
${MATCHMOVER_INCLUDE_DIR}
${SDL_INCLUDE_DIR}
${GTKMM_INCLUDE_DIRS}
)


add_executable(matchmover ${MATCHMOVER_SOURCES_FILES})  
target_link_libraries(matchmover ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${SDL_LIBRARY} ${GTKMM_LIBRARIES})  


set(SOURCE_FILES main.cpp)
add_executable(main ${SOURCE_FILES})
#GLFW3
#target_link_libraries(main slamAR glfw3 GL m Xrandr Xi X11 Xxf86vm pthread dl Xinerama Xcursor)
#glut
target_link_libraries(main slamAR -lGL -lglut)

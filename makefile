all: preintegration.cc \
		 ./bfvio/preintegration.h \
		 ./bfvio/preintegration.cpp \
		 ./bfvio/kinematics.h \
		 ./bfvio/kinematics.cpp \
		 ./bfvio/slam_trajectory_drawer.h \
		 ./bfvio/slam_trajectory_drawer.cpp \
		 ./bfvio/utils.h \
		 ./bfvio/utils.cpp
	g++ -std=c++17 -ggdb preintegration.cc \
			./bfvio/preintegration.cpp \
			./bfvio/kinematics.cpp \
			./bfvio/slam_trajectory_drawer.cpp \
			./bfvio/utils.cpp \
			-isystem/usr/include/eigen3 \
			-lstdc++fs \
			-lGL -lGLU -lGLEW -lglut \
			-o preintegration

clean:
	rm preintegration


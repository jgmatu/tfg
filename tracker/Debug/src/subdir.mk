################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Categories.cpp \
../src/HSLTool.cpp \
../src/HSVHistogram.cpp \
../src/MapPlobs.cpp \
../src/OctoMap.cpp \
../src/OctoMaps.cpp \
../src/Pblob.cpp \
../src/PlobTracker.cpp \
../src/node_tracker.cpp 

OBJS += \
./src/Categories.o \
./src/HSLTool.o \
./src/HSVHistogram.o \
./src/MapPlobs.o \
./src/OctoMap.o \
./src/OctoMaps.o \
./src/Pblob.o \
./src/PlobTracker.o \
./src/node_tracker.o 

CPP_DEPS += \
./src/Categories.d \
./src/HSLTool.d \
./src/HSVHistogram.d \
./src/MapPlobs.d \
./src/OctoMap.d \
./src/OctoMaps.d \
./src/Pblob.d \
./src/PlobTracker.d \
./src/node_tracker.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/opt/ros/kinetic/include -I/home/javi/catkin_ws/devel/include -I/usr/include/libglademm-2.4 -I/opt/ros/kinetic/include/opencv-3.2.0-dev -I/usr/include/eigen3 -I/usr/include/pcl-1.7 -I/usr/include/gtkmm-3.0 -O0 -g3 -Wall -std=c++11 -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



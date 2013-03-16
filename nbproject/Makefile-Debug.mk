#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=avr-gcc
CCC=avr-g++
CXX=avr-g++
FC=gfortran
AS=avr-as

# Macros
CND_PLATFORM=Arduino-Windows
CND_DLIB_EXT=dll
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/main.o


# C Compiler Flags
CFLAGS=-gstabs+ -Wall -Os -ffunction-sections -fdata-sections -mmcu=atmega328p

# CC Compiler Flags
CCFLAGS=-gstabs+ -Wall -Os -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p
CXXFLAGS=-gstabs+ -Wall -Os -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=../arduino_corelib/dist/Debug/Arduino-Windows/libarduino_corelib.a -lm

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/esc_test.exe

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/esc_test.exe: ../arduino_corelib/dist/Debug/Arduino-Windows/libarduino_corelib.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/esc_test.exe: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	avr-gcc.exe -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/esc_test ${OBJECTFILES} ${LDLIBSOPTIONS} -gstabs+ -Os -Wl,--gc-sections -mmcu=atmega328p

${OBJECTDIR}/main.o: main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DARDUINO=103 -DF_CPU=16000000UL -D_DEBUG_ -D__AVR_ATmega328P__ -I../arduino_corelib -gstabs+ -Wall -Os -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p -MMD -MP -MF $@.d -o ${OBJECTDIR}/main.o main.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/esc_test.exe

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc

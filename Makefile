#############################################################################
# Makefile for building: serial
# Generated by qmake (2.01a) (Qt 4.5.3) on: Thu Jul 7 16:02:58 2016
# Project:  serial.pro
# Template: app
# Command: /usr/local/Qt/bin/qmake -spec /usr/local/Qt/mkspecs/qws/linux-arm-g++ -unix -o Makefile serial.pro
#############################################################################

####### Compiler, tools and options

CC            = arm-none-linux-gnueabi-gcc
CXX           = arm-none-linux-gnueabi-g++
DEFINES       = 
CFLAGS        = -pipe -O2 -Wall -W $(DEFINES)
CXXFLAGS      = -pipe -O2 -Wall -W $(DEFINES)
INCPATH       = -I/usr/local/Qt/mkspecs/qws/linux-arm-g++ -I.
LINK          = arm-none-linux-gnueabi-g++
LFLAGS        = -Wl,-O1 -Wl,-rpath,/usr/local/Qt/lib
LIBS          = $(SUBLIBS)   -L/home/baijie/work/common_tool/serial/../../../../../../../usr/lib/i386-linux-gnu/ -lpthread
AR            = arm-none-linux-gnueabi-ar cqs
RANLIB        = 
QMAKE         = /usr/local/Qt/bin/qmake
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -sf
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = ./

####### Files

SOURCES       = serial.cpp \
		serial_test.cpp \
		usage.cpp 
OBJECTS       = serial.o \
		serial_test.o \
		usage.o
DIST          = /usr/local/Qt/mkspecs/common/g++.conf \
		/usr/local/Qt/mkspecs/common/unix.conf \
		/usr/local/Qt/mkspecs/common/linux.conf \
		/usr/local/Qt/mkspecs/common/qws.conf \
		/usr/local/Qt/mkspecs/qconfig.pri \
		/usr/local/Qt/mkspecs/features/qt_functions.prf \
		/usr/local/Qt/mkspecs/features/qt_config.prf \
		/usr/local/Qt/mkspecs/features/exclusive_builds.prf \
		/usr/local/Qt/mkspecs/features/default_pre.prf \
		/usr/local/Qt/mkspecs/features/release.prf \
		/usr/local/Qt/mkspecs/features/default_post.prf \
		/usr/local/Qt/mkspecs/features/warn_on.prf \
		/usr/local/Qt/mkspecs/features/resources.prf \
		/usr/local/Qt/mkspecs/features/uic.prf \
		/usr/local/Qt/mkspecs/features/yacc.prf \
		/usr/local/Qt/mkspecs/features/lex.prf \
		/usr/local/Qt/mkspecs/features/include_source_dir.prf \
		serial.pro
QMAKE_TARGET  = serial
DESTDIR       = 
TARGET        = serial

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

Makefile: serial.pro  /usr/local/Qt/mkspecs/qws/linux-arm-g++/qmake.conf /usr/local/Qt/mkspecs/common/g++.conf \
		/usr/local/Qt/mkspecs/common/unix.conf \
		/usr/local/Qt/mkspecs/common/linux.conf \
		/usr/local/Qt/mkspecs/common/qws.conf \
		/usr/local/Qt/mkspecs/qconfig.pri \
		/usr/local/Qt/mkspecs/features/qt_functions.prf \
		/usr/local/Qt/mkspecs/features/qt_config.prf \
		/usr/local/Qt/mkspecs/features/exclusive_builds.prf \
		/usr/local/Qt/mkspecs/features/default_pre.prf \
		/usr/local/Qt/mkspecs/features/release.prf \
		/usr/local/Qt/mkspecs/features/default_post.prf \
		/usr/local/Qt/mkspecs/features/warn_on.prf \
		/usr/local/Qt/mkspecs/features/resources.prf \
		/usr/local/Qt/mkspecs/features/uic.prf \
		/usr/local/Qt/mkspecs/features/yacc.prf \
		/usr/local/Qt/mkspecs/features/lex.prf \
		/usr/local/Qt/mkspecs/features/include_source_dir.prf
	$(QMAKE) -spec /usr/local/Qt/mkspecs/qws/linux-arm-g++ -unix -o Makefile serial.pro
/usr/local/Qt/mkspecs/common/g++.conf:
/usr/local/Qt/mkspecs/common/unix.conf:
/usr/local/Qt/mkspecs/common/linux.conf:
/usr/local/Qt/mkspecs/common/qws.conf:
/usr/local/Qt/mkspecs/qconfig.pri:
/usr/local/Qt/mkspecs/features/qt_functions.prf:
/usr/local/Qt/mkspecs/features/qt_config.prf:
/usr/local/Qt/mkspecs/features/exclusive_builds.prf:
/usr/local/Qt/mkspecs/features/default_pre.prf:
/usr/local/Qt/mkspecs/features/release.prf:
/usr/local/Qt/mkspecs/features/default_post.prf:
/usr/local/Qt/mkspecs/features/warn_on.prf:
/usr/local/Qt/mkspecs/features/resources.prf:
/usr/local/Qt/mkspecs/features/uic.prf:
/usr/local/Qt/mkspecs/features/yacc.prf:
/usr/local/Qt/mkspecs/features/lex.prf:
/usr/local/Qt/mkspecs/features/include_source_dir.prf:
qmake:  FORCE
	@$(QMAKE) -spec /usr/local/Qt/mkspecs/qws/linux-arm-g++ -unix -o Makefile serial.pro

dist: 
	@$(CHK_DIR_EXISTS) .tmp/serial1.0.0 || $(MKDIR) .tmp/serial1.0.0 
	$(COPY_FILE) --parents $(SOURCES) $(DIST) .tmp/serial1.0.0/ && (cd `dirname .tmp/serial1.0.0` && $(TAR) serial1.0.0.tar serial1.0.0 && $(COMPRESS) serial1.0.0.tar) && $(MOVE) `dirname .tmp/serial1.0.0`/serial1.0.0.tar.gz . && $(DEL_FILE) -r .tmp/serial1.0.0


clean:compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile


compiler_rcc_make_all:
compiler_rcc_clean:
compiler_uic_make_all:
compiler_uic_clean:
compiler_image_collection_make_all: qmake_image_collection.cpp
compiler_image_collection_clean:
	-$(DEL_FILE) qmake_image_collection.cpp
compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: 

####### Compile

serial.o: serial.cpp serial.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o serial.o serial.cpp

serial_test.o: serial_test.cpp serial.h \
		usage.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o serial_test.o serial_test.cpp

usage.o: usage.cpp usage.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o usage.o usage.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:


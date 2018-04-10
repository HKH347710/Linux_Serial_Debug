TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += \
    serial.cpp \
    serial_test.cpp \
    usage.cpp

HEADERS += \
    serial.h \
    usage.h

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../../usr/lib/i386-linux-gnu/release/ -lpthread
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../../usr/lib/i386-linux-gnu/debug/ -lpthread
else:symbian: LIBS += -lpthread
else:unix: LIBS += -L$$PWD/../../../../../../../usr/lib/i386-linux-gnu/ -lpthread

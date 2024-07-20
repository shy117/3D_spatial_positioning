QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    cloudviewerthread.cpp \
    main.cpp \
    mainwindow.cpp \
    method.cpp

HEADERS += \
    cloudviewerthread.h \
    mainwindow.h \
    method.h \
    stereoconfig.h

FORMS += \
    mainwindow.ui

QMAKE_LFLAGS += -no-pie


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


INCLUDEPATH += /usr/local/include/opencv4
LIBS += /usr/local/lib/libopencv_*.so

INCLUDEPATH += /usr/local/include/eigen3/

INCLUDEPATH += /usr/include/vtk-6.3
LIBS += /usr/lib/x86_64-linux-gnu/libvtk*.so

INCLUDEPATH += /usr/include/pcl-1.8
LIBS += /usr/lib/x86_64-linux-gnu/libpcl_*.so

INCLUDEPATH += /usr/include/boost
LIBS += /usr/lib/x86_64-linux-gnu/libboost_*.so

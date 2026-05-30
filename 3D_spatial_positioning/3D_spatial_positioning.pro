QT += core gui widgets

CONFIG += c++17
TEMPLATE = app
TARGET = 3D_spatial_positioning

PROJECT_ROOT = $$clean_path($$PWD/..)

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    method.cpp

HEADERS += \
    mainwindow.h \
    method.h \
    stereoconfig.h

FORMS += \
    mainwindow.ui

unix:!android {
    DESTDIR = $$PROJECT_ROOT/build/ubuntu-qmake/bin
    OBJECTS_DIR = $$PROJECT_ROOT/build/ubuntu-qmake/obj
    MOC_DIR = $$PROJECT_ROOT/build/ubuntu-qmake/moc
    RCC_DIR = $$PROJECT_ROOT/build/ubuntu-qmake/rcc
    UI_DIR = $$PROJECT_ROOT/build/ubuntu-qmake/ui

    QMAKE_LFLAGS += -no-pie
    DEFINES += HAVE_PCL

    SOURCES += cloudviewerthread.cpp
    HEADERS += cloudviewerthread.h

    INCLUDEPATH += /usr/local/include/opencv4
    LIBS += /usr/local/lib/libopencv_*.so

    INCLUDEPATH += /usr/include/vtk-6.3
    LIBS += /usr/lib/x86_64-linux-gnu/libvtk*.so

    INCLUDEPATH += /usr/include/pcl-1.8
    LIBS += /usr/lib/x86_64-linux-gnu/libpcl_*.so

    INCLUDEPATH += /usr/include/boost
    LIBS += /usr/lib/x86_64-linux-gnu/libboost_*.so

    target.path = /opt/$${TARGET}/bin
    INSTALLS += target
}

win32 {
    DESTDIR = $$PROJECT_ROOT/build/windows-qt6-mingw/bin
    OBJECTS_DIR = $$PROJECT_ROOT/build/windows-qt6-mingw/obj
    MOC_DIR = $$PROJECT_ROOT/build/windows-qt6-mingw/moc
    RCC_DIR = $$PROJECT_ROOT/build/windows-qt6-mingw/rcc
    UI_DIR = $$PROJECT_ROOT/build/windows-qt6-mingw/ui

    DEFINES += _CRT_SECURE_NO_WARNINGS NOMINMAX

    # 本机当前可用 OpenCV 包。若迁移到其他机器，只需改这里。
    OPENCV_DIR = E:/Workspace/MyGit/videoCapture/videoCapture/3rdparty/opencv

    INCLUDEPATH += $$OPENCV_DIR/include

    LIBS += -L"$$OPENCV_DIR/lib"
    LIBS += \
        -lopencv_core401 \
        -lopencv_imgproc401 \
        -lopencv_imgcodecs401 \
        -lopencv_videoio400 \
        -lopencv_calib3d401 \
        -lopencv_highgui401 \
        -lopencv_ximgproc401
}

qnx: target.path = /tmp/$${TARGET}/bin

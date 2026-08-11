QT += core gui widgets
TARGET = scara_panel
TEMPLATE = app

SOURCES += main.cpp mainwindow.cpp i2c_worker.cpp sensor_sim.c i2c_driver.c camera.cpp
HEADERS += mainwindow.h i2c_worker.h sensor_sim.h i2c_driver.h kinematics.h camera.h
FORMS += mainwindow.ui

QMAKE_CFLAGS += -std=c99

# OpenCV (龙芯交叉编译)
OPENCV_CROSS = /opt/sdk-loongson/LoongOS/v0.1/sysroots/loongarch64-Loongson-linux/usr
INCLUDEPATH += $$OPENCV_CROSS/include/opencv4
LIBS += -L$$OPENCV_CROSS/lib
LIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lm

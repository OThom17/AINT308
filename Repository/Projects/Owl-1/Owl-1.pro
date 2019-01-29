TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


INCLUDEPATH += E:\openCV314\build\include ### was c:/opencv31/release/install/include

#LIBS += -LC:\opencv31\build\install\x86\mingw\lib
#LIBS += -LC:\opencv31\build\install\x86\mingw\bin
#LIBS += -LC:\openCV314\build\lib\Release
LIBS += -E:\openCV314\build\lib

LIBS +=    -lopencv_core345 \
    -lopencv_highgui345 \
    -lopencv_imgproc345 \
    -lopencv_features2d345 \
    -lopencv_calib3d345 \
    -lopencv_videoio345d \
    -lws2_32 \
##    -lopencv_ffmpeg310

SOURCES += \
    ../../Sources/Owl-1/main.cpp \
    ../../Sources/Owl-1/stereocontrol.cpp

HEADERS += \
    ../../Sources/Owl-1/owl-comms.h \
    ../../Sources/Owl-1/owl-cv.h \
    ../../Sources/Owl-1/owl-pwm.h

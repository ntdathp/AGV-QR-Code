QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    TCP.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    TCP.h \
    mainwindow.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/build-sharedlib-compilation-Desktop_Qt_6_5_3_MinGW_64_bit-Release/release/ -lqcustomplot2
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/build-sharedlib-compilation-Desktop_Qt_6_5_3_MinGW_64_bit-Release/debug/ -lqcustomplot2
else:unix: LIBS += -L$$PWD/build-sharedlib-compilation-Desktop_Qt_6_5_3_MinGW_64_bit-Release/ -lqcustomplot2

INCLUDEPATH += $$PWD/build-sharedlib-compilation-Desktop_Qt_6_5_3_MinGW_64_bit-Release/release
DEPENDPATH += $$PWD/build-sharedlib-compilation-Desktop_Qt_6_5_3_MinGW_64_bit-Release/release

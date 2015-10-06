TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.c \
    libs/7seg-driver/indicator.c \
    libs/encoder/encoder.c \
    libs/IND_Driver/indicator.c \
    libs/IND_Driver/spi.c \
    main_charger.c \
    indicator.c \
    periphery.c \
    test.c

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    libs/7seg-driver/indicator.h \
    libs/encoder/encoder.h \
    main.h \
    libs/IND_Driver/indicator.h \
    libs/IND_Driver/compilers.h \
    libs/IND_Driver/spi.h


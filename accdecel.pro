QT += charts

CONFIG += c++14

HEADERS += \
    Motion.hpp

SOURCES += \
    \ #main-chart.cpp  \
    main-stdout.cpp

#CONFIG += static staticlib static-runtime
#DEFINES += QT_NODLL

gcc:QMAKE_CXXFLAGS += -Wno-deprecated-declarations

target.path = $$[QT_INSTALL_EXAMPLES]/charts/multiaxis
INSTALLS += target

TEMPLATE = app
CONFIG   = qt

HEADERS += \
        RMapViewer.h \
        RMap.h \
        RRangeScan.h \
        RPathPlan.h \
        RPathPlan_astar.h \
        RPathPlan_astarDyn.h

SOURCES += \
        main.cpp \
        RMapViewer.cpp \
        RMap.cpp \
        RRangeScan.cpp \
        RPathPlan.cpp \
        RPathPlan_astar.cpp \
        RPathPlan_astarDyn.cpp

TARGET = pathplan_gui

#QMAKE_CXXFLAGS += -g -rdynamic

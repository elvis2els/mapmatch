TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    ivmm/ivmm.cpp \
    map-match/map_match.cpp \
    generalmap.cpp \
    gps.cpp \
    roadmap.cpp \
    time_estimate.cpp

HEADERS += \
    ivmm/ivmm.h \
    generalmap.h \
    geomerty.h \
    gps.h \
    map_graph_property.hpp \
    roadmap.h \
    time_estimate.h \
    traits.hpp \
    util.h \
    bj-road-epsg3785/bj_road_epsg3785.h \
    sutil/boost/date_time/date_time_format.hpp \
    sutil/boost/geometry/distance_projected_point_return_point.hpp \
    sutil/boost/graph/breadth_first_search_to_dest.hpp \
    sutil/boost/graph/dijkstra_shortest_paths_to_dest.hpp \
    sutil/boost/graph/lazy_associative_property_map.hpp \
    sutil/boost/graph/visitor_adaptor.hpp \
    sutil/boost/range/extend.hpp \
    sutil/cow_pimpl.hpp \
    sutil/key_visitor.hpp \
    sutil/simple_guard.hpp \
    sutil/simple_progress.hpp

DISTFILES += \
    sutil/README.md

INCLUDEPATH += /usr/include/mysql
LIBS += -lshp -lmysqlpp -lboost_system -lboost_filesystem -lboost_program_options -lboost_thread -lboost_date_time

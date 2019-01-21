cmake_minimum_required (VERSION 2.6)
project (test_filter)
include_directories(/home/shuo/debug_ransac_with_python)
include_directories(home/shuo/debug_ransac_with_python/dronerace)
include_directories("/usr/include/")

set(HEADERS std.h
    dronerace/control.h
    dronerace/dronerace.h
    dronerace/fifo.h
    dronerace/filter.h
    dronerace/flightplan.h
    dronerace/ransac_run.h
    math/pprz_algebra.h
    math/pprz_algebra.h
    math/pprz_matrix_decomp_float.h
    math/RANSAC.h
    )

add_definitions(-D_CRT_SECURE_NO_WARNINGS)
add_library(test_filter SHARED processlog.c dronerace/filter.c dronerace/fifo.c dronerace/ransac_run.c
    dronerace/flightplan.c dronerace/control.c math/RANSAC.c math/pprz_matrix_decomp_float.c )
target_link_libraries(test_filter m)

INSTALL(TARGETS test_filter
    LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/test_filter)


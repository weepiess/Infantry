include(FindPackageHandleStandardArgs)

set(GLOG_ROOT_DIR "/usr/local/")

find_path(GLOG_INCLUDE_DIR logging.h
        PATHS ${GLOG_ROOT_DIR}/include/glog/)

find_library(GLOG_LIBRARY glog
        PATHS ${GLOG_ROOT_DIR}/lib
        PATH_SUFFIXES lib lib64)

find_package_handle_standard_args(Glog DEFAULT_MSG GLOG_INCLUDE_DIR GLOG_LIBRARY)

if(GLOG_FOUND)
    message(STATUS "Found glog (include: ${GLOG_INCLUDE_DIR}, library: ${GLOG_LIBRARY})")
    mark_as_advanced(GLOG_ROOT_DIR GLOG_LIBRARY_RELEASE GLOG_LIBRARY_DEBUG
                                 GLOG_LIBRARY GLOG_INCLUDE_DIR)
endif()
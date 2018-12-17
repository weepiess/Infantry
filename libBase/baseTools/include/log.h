#ifndef LIBBASE_LOG_H
#define LIBBASE_LOG_H

#include "glog/logging.h"
#include "glog/raw_logging.h"
#include <fstream>

#define LOG_INFO LOG(INFO)
#define LOG_WARNING LOG(WARNING)
#define LOG_ERROR LOG(ERROR)
#define LOG_FATAL LOG(FATAL)

#define LOG_INFO_IF(condition) LOG_IF(INFO,condition)
#define LOG_WARNING_IF(condition) LOG_IF(WARNING,condition)
#define LOG_ERROR_IF(condition) LOG_IF(ERROR,condition)
#define LOG_FATAL_IF(condition) LOG_IF(FATAL,condition)

#define LOG_INFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define LOG_WARNING_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define LOG_ERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#define DLOG_INFO DLOG(INFO)
#define DLOG_WARNING DLOG(WARNING)

#define LOG_WARNING_FIRST_N(times) LOG_FIRST_N(WARNING, times)

#define NOTICE(text) {               \
  static bool flag = true;           \
  if(flag) {                         \
    std::cout << text << std::endl;  \
    flag = false;                    \
  }                                  \
}                                    \

class GLogWrapper {
public:
    GLogWrapper(char* program){
        google::InitGoogleLogging(program);
        FLAGS_stderrthreshold=google::INFO;
        FLAGS_colorlogtostderr=true;
        FLAGS_v = 3;
        google::InstallFailureSignalHandler();
    }

    ~GLogWrapper(){
        google::ShutdownGoogleLogging();
    }
};

#endif // LIBBASE_LOG_H
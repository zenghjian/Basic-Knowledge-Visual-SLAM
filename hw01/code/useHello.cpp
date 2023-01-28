#include "hello.h"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>
DEFINE_int64(print_times,1,"how many sayHello should be printed");


int main( int argc, char** argv ) {

  google::ParseCommandLineFlags(&argc, &argv, true);
  EXPECT_GE(FLAGS_print_times, 0);
  google::InitGoogleLogging(argv[0]);
  while(FLAGS_print_times != 0){
    sayHello();
    --FLAGS_print_times;
  }
  google::ShutdownGoogleLogging();

  return 0;
}

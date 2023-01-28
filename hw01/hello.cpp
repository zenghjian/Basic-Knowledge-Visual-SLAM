#include "hello.h"
#include <iostream>
#include <glog/logging.h>


void sayHello() {
    // std::cout<<"Hello SLAM"<<std::endl;
    // LOG(INFO) << "Hello SLAM" ;       //输出一个Info日志
    // LOG(WARNING) << "Hello SLAM";  //输出一个Warning日志
    LOG(ERROR) << "Hello SLAM";      //输出一个Error日志    
    }

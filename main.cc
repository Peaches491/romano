#include <iostream>

#include "glog/logging.h"

int main(int argc, char** argv) {
  ::google::InitGoogleLogging(argv[0]);
  LOG(INFO) << "Romano - Hello, world!";
}


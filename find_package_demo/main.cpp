#include <iostream>
#include <glog/logging.h>
int main() {
    std::cout << "Hello, World!" << std::endl;
    LOG(ERROR) << "test log";
    return 0;
}
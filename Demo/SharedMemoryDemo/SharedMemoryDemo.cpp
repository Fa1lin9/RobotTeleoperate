#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <iostream>
#include <thread>
#include <chrono>

using namespace boost::interprocess;

struct SharedData {
    int flag;
    int value;
};

int main() {

}

// src/main.cpp
#include "seecam.hpp"

int main() {
    setenv("RUST_LOG", "debug", 1);
    SeeCam seecam;
    seecam.run();
    return 0;
}
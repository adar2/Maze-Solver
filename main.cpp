#include <iostream>
#include "Algorithms/utils.h"




int main(int argc, char *argv[]) {

    if (argc < 2) {
        std::cout << "No arguments, exiting." << std::endl;
        exit(1);
    }
    parse_file(argv[1]);
    return 0;
}

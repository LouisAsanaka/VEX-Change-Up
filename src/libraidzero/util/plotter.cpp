#include "libraidzero/util/plotter.hpp"

#include <sstream>
#include <string>
#include <iostream>
#include <iomanip>

void plotterStart() {
    std::cout << "{START}" << std::endl;
}

void plotterPlot(const std::vector<double>& elements) {
    /*
     * Takes vector of elements to print to plotter
     * {x_data, y1, y2, etc.}
     */
    std::stringstream ss;
    ss << "{";

    size_t size = elements.size();
    for (unsigned i = 0; i < size; i++) {
        ss << std::fixed << std::setprecision(4) << elements.at(i);
        if (i != size - 1) {
            ss << ",";
        }
    }
    ss << "}";
    std::cout << ss.str() << " " << std::endl;
}

void plotterStop() {
    std::cout << "{STOP}" << std::endl;
}

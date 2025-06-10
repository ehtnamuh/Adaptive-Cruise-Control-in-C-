#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <sstream>


std::map<std::string, double> loadParameters(const std::string& filename) {
    std::ifstream infile(filename);
    std::map<std::string, double> parameters;
    std::string line, key;
    double value;

    if (!infile) {
        std::cerr << "Unable to open file " << filename;
        return parameters;
    }

    while (std::getline(infile, line)) {
        std::istringstream ss(line);
        if (std::getline(ss, key, ',') && ss >> value) {
            parameters[key] = value;
        }
    }

    infile.close();
    return parameters;
}
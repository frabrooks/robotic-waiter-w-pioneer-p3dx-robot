#include "Structures.h"
#include "Utils.h"

#include <cstdlib> 
#include <fstream>
#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <vector>

//namespace Utils {

    //template<typename Out>
    void split(const std::string &s, char delim, Out result) {
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim)) {
            *(result++) = item;
        }
    }
    std::vector<std::string> split(const std::string &s, char delim) {
        std::vector<std::string> elems;
        split(s, delim, std::back_inserter(elems));
        return elems;
    }

    string intToString(int value) {
        stringstream stream;

        stream << value;
        
        return stream.str();
    }
//}

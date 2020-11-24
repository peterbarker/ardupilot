#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <istream>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <iostream>
#include <map>

class CSVReader
{
public:
    CSVReader() {}
    bool open(const char *filename) {
        s.open(filename);
        prime_column_headings();
        return true;
    };

    void prime_column_headings() {
        auto vec = getNextLineAndSplitIntoTokens();
        uint8_t count = 0;
        for (std::string &item : vec) {
            label_to_offset.insert(std::pair<std::string, uint8_t>(item, count));
            count++;
        }
    }

    bool next_line() WARN_IF_UNUSED {
        current_line = getNextLineAndSplitIntoTokens();
        if (current_line.size() == 1) {  // FIXME
            return false;
        }
        return true;
    }

    double get_value(const std::string label) {
        const uint8_t offset = label_to_offset.at(label);
        return std::stod(current_line[offset]);
    }

private:
    std::ifstream s;


    // function to read the CSV file line by line
    std::vector<std::string> getNextLineAndSplitIntoTokens() {
        std::string line;
        std::getline(s, line);

        std::stringstream lineStream(line);

        std::string cell;
        std::vector<std::string> result;
        while(std::getline(lineStream,cell, ';'))
        {
            result.push_back(cell);
        }
        // This checks for a trailing comma with no data after it.
        if (!lineStream && cell.empty())
        {
            // If there was a trailing comma then add an empty element.
            result.push_back("");
        }
        return result;
    }

    std::vector<std::string> current_line;

    std::map<std::string, uint8_t> label_to_offset;
};



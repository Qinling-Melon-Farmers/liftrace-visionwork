#pragma once
#include <fstream>
#include <string>
#include <unistd.h>

// Reads/access checks only: never export, configure, enable or change duty.
struct SysfsReadChecks {
    bool read(const std::string& path, std::string& value) {
        std::ifstream input(path);
        std::string extra;
        return bool(input >> value) && !(input >> extra);
    }
    bool writable(const std::string& path) {
        return access(path.c_str(), W_OK) == 0;
    }
};

template<class ReadChecks>
bool checkedPassiveConfiguration(ReadChecks& checks, const std::string& path) {
    const char* fields[] = {"period", "polarity", "enable"};
    const char* expected[] = {"20000000", "normal", "0"};
    for (int i=0; i<3; ++i) {
        std::string actual;
        if (!checks.read(path + "/" + fields[i], actual) || actual != expected[i])
            return false;
    }
    const char* writable[] = {"period", "duty_cycle", "polarity", "enable"};
    for (const char* field : writable)
        if (!checks.writable(path + "/" + field)) return false;
    return true;
}

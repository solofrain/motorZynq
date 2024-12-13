#pragma once

#include <iostream>
#include <iomanip>

inline auto cout_hex = [](int a) {
    return std::hex << a << std::dec;
};

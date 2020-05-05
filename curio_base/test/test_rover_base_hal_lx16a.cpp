/// \brief tests utils used in rover_base_hal_lx16a

#include <algorithm>
#include <iostream>
#include <string>

/// \brief simple case insensitive comparison
///
/// See:
/// https://stackoverflow.com/questions/11635/case-insensitive-string-comparison-in-c
/// https://en.cppreference.com/w/cpp/string/byte/tolower
bool iequal(const std::string& a, const std::string& b)
{
    if (a.size() != b.size()) return false;
    bool is_equal = true;
    for (auto sa = a.begin(), sb = b.begin();
        sa != a.end() && sb != b.end();
        ++sa, ++sb)
    {
        is_equal &= (std::tolower(static_cast<unsigned char>(*sa)) 
            == std::tolower(static_cast<unsigned char>(*sb)));
    }
    return is_equal;

    // C++14
    // return std::for_each(a.begin(), a.end(), 
    //     [](char a, char b) {
    //         return tolower(static_cast<unsigned char>(a)) == tolower(static_cast<unsigned char>(b));
    //     });
}    

int main(int argc, const char* argv[])
{
    std::string apple("apple");
    std::string Apple("Apple");
    std::string orange("orange");
    std::string Orange("Orange");

    std::cout << apple << " == " << apple 
        << ": " << iequal(apple, apple) << std::endl;
    std::cout << apple << " == " << orange 
        << ": " << iequal(apple, orange) << std::endl;
    std::cout << apple << " == " << Apple 
        << ": " << iequal(apple, Apple) << std::endl;
    std::cout << orange << " == " << Orange 
        << ": " << iequal(orange, Orange) << std::endl;

    std::string list10("0123456789");
    std::string list11("01234567890");
    std::cout << list10 << " == " << list11 
        << ": " << iequal(list10, list11) << std::endl;

    std::string left("left");
    std::string LEFT("LEFT");
    std::string right("right");
    std::cout << left << " == " << right 
        << ": " << iequal(left, right) << std::endl;
    std::cout << left << " == " << LEFT 
        << ": " << iequal(left, LEFT) << std::endl;

    return 0;
}
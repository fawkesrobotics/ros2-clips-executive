// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef CX_UTILS__STRINGUTILS_HPP
#define CX_UTILS__STRINGUTILS_HPP

#include <algorithm>
#include <iostream>
#include <random>
#include <string>

using std::string;

namespace cx {

std::string chars{
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890-"};
std::random_device rd;
std::mt19937 generator(rd());

std::string generate_rnd_string(int length) {
  std::string output(chars);
  std::shuffle(output.begin(), output.end(), generator);
  return output.substr(0, length);
}

} // namespace cx
#endif // !CX_UTILS__STRINGUTILS_HPP

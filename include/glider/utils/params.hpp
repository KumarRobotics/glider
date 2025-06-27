/*
* April 2025
*
* Load params from a file
*/

#pragma once

#include <typeinfo>
#include <tuple>
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>

struct Params
{
    Params() = default;

    template <typename T>
    std::map<std::string, T> load(std::string path);   
};

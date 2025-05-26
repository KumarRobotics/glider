/*
*
*
*/
#include <sstream>
#include "glider/utils/params.hpp"

template <typename T>
std::map<std::string, T> Params::load(std::string path)
{
    YAML::Node config = YAML::LoadFile(path);
    
    std::string type_key;
    for (const auto& pair : config)
    {
        type_key = pair.first.as<std::string>();
    }  

    

    std::string type_str = typeid(T).name();
    if (type_str == "d") type_str = "double";
    if (type_str == "f") type_str = "float";
    if (type_str == "i") type_str = "int";

    if (type_str != type_key) 
    {
        std::stringstream error_str;
        error_str << "The type you passed is not equal to that in the yaml file, git type " << type_str << ", expected type " << type_key;
        throw std::runtime_error(error_str.str());
    }

    std::map<std::string, T> params;
    for (const auto& pair : config[type_key])
    {
        params[pair.first.as<std::string>()] = pair.second.as<T>();
    }

    return params;
}

template std::map<std::string, double> Params::load<double>(std::string);
template std::map<std::string, float> Params::load<float>(std::string);
template std::map<std::string, int> Params::load<int>(std::string);

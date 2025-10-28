#pragma once

#include "mapek/util/definitions.hpp"
#include <string>
#include <cassert>

// TODO merge with general_definitions
 
namespace param_utils{

    std::string getDataAsString(const ParamValue& val);

    template <typename T>
    T getData(const ParamValue& val);

    ParamValue getParamValue(const double& data);
    ParamValue getParamValue(const int& data);
    ParamValue getParamValue(const bool& data);
    ParamValue getParamValue(const std::string& data);
    ParamValue getParamValue(const char* data);


}//namespace param_utils
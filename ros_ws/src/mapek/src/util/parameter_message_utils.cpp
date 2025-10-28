#include "mapek/util/parameter_message_utils.hpp"
#include "mapek/util/string_utils.hpp"

namespace param_utils{


    template <>
    double getData<double>(const ParamValue& val){
        assert(val.type == ParamType::PARAMETER_DOUBLE or val.type == ParamType::PARAMETER_INTEGER);
        
        if (val.type == ParamType::PARAMETER_DOUBLE)
            return val.double_value;
        else
            return val.integer_value;
    }

    template <>
    int getData<int>(const ParamValue& val){
        assert(val.type == ParamType::PARAMETER_INTEGER);
        return val.integer_value;
    }

    template <>
    bool getData<bool>(const ParamValue& val){
        assert(val.type == ParamType::PARAMETER_BOOL);
        return val.bool_value;
    }

    template <>
    std::string getData<std::string>(const ParamValue& val){
        assert(val.type == ParamType::PARAMETER_STRING);
        return val.string_value;
    }

    ParamValue getParamValue(const double& data){
        ParamValue val;
        val.type = ParamType::PARAMETER_DOUBLE;
        val.double_value = data;
        return val;
    }

    ParamValue getParamValue(const int& data){
        ParamValue val;
        val.type = ParamType::PARAMETER_INTEGER;
        val.integer_value = data;
        return val;
    }

    ParamValue getParamValue(const bool& data){
        ParamValue val;
        val.type = ParamType::PARAMETER_BOOL;
        val.bool_value = data;
        return val;
    }

    ParamValue getParamValue(const std::string& data){
        ParamValue val;
        val.type = ParamType::PARAMETER_STRING;
        val.string_value = data;
        return val;
    }

    ParamValue getParamValue(const char* data){
        std::string str_data = data;
        return getParamValue(str_data); 
    }

    std::string getDataAsString(const ParamValue& val){
        // retrun --> no break needed
        switch (val.type){
            case(ParamType::PARAMETER_DOUBLE):
                return string_utils::to_string(getData<double>(val));
            case(ParamType::PARAMETER_INTEGER):
                return string_utils::to_string((double)getData<int>(val));
            case(ParamType::PARAMETER_BOOL):
                return string_utils::to_string(getData<bool>(val));
            case(ParamType::PARAMETER_STRING):
                return string_utils::to_string(getData<std::string>(val));
            default:
                throw std::runtime_error("unsupport param type");
        }
    }

}
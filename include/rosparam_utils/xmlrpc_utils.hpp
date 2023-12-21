#pragma once

#include "ros/ros.h"

#include "string"

using std::string;

namespace xmlrpc_utils
{
    /**
     * @brief Checks if the parameter's name matches a given parameter name and parses the value of the parameter 
     * in the output if its the case.
     * @param parameter Struct parameter from a config that contains a name and a XmlRpcValue.
     * @param parameter_name_to_check_against String to match the parameter name against.
     * @param output Reference to an output that is set to the parameter's value if their is a match. Not modify otherwise.
     * @tparam Type of the variable to parse from the parameter.
    */
    template <typename T>
    bool paramMatchAndParse(const XmlRpc::XmlRpcValue parameter, const string& parameter_name_to_check_against, T& output);

    template <>
    bool paramMatchAndParse<string>(const XmlRpc::XmlRpcValue parameter, const string& parameter_name_to_check_against, string& output)
    {
        auto parameter_it = parameter.begin();

        const string& parameter_name = parameter_it->first;
        const XmlRpc::XmlRpcValue& parameter_value = parameter_it->second;
        
        if (parameter_name == parameter_name_to_check_against && parameter_value.getType() ==
            XmlRpc::XmlRpcValue::TypeString)
        {
            output= static_cast<string>(parameter_value);
            return true;
        }
        else
        {
            return false;
        }
    }

    template<>
    bool paramMatchAndParse<float>(const XmlRpc::XmlRpcValue parameter, const string& parameter_name_to_check_against, float& output)
    {
        auto parameter_it = parameter.begin();

        const string& parameter_name = parameter_it->first;
        const XmlRpc::XmlRpcValue& parameter_value = parameter_it->second;
        
        if (parameter_name == parameter_name_to_check_against && parameter_value.getType() ==
            XmlRpc::XmlRpcValue::TypeDouble)
        {
            output  = static_cast<float>(static_cast<double>(parameter_value));
            return true;
        }
        else
        {
            return false;
        }
    }
};
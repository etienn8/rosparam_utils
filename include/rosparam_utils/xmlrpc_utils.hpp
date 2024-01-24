#pragma once

#include "ros/ros.h"

#include <string>
#include <vector>
#include <utility>

#include "parameter_package_fetch_struct.hpp"

using std::string;
using std::vector;

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

    /*template <>
    bool paramMatchAndParse<string>(const XmlRpc::XmlRpcValue parameter, const string& parameter_name_to_check_against, string& output);

    template<>
    bool paramMatchAndParse<float>(const XmlRpc::XmlRpcValue parameter, const string& parameter_name_to_check_against, float& output);*/

    /**
     * @brief Create a list of parameter structures where each is element is populated from a list of list of ROS parameters 
     * based on a given structure. It allows to parse an arbitrary long list of list based on given parameters. Parameters that 
     * don't match an expected parameter or don't have the expected type will be ignored. 
     * @param nh ROS NodeHandle to have access to the parameters.
     * @param name_of_the_node Name of the node. Only used for logging.
     * @param name_of_list Name of the list of list of parameters from a ROS config file.
     * @param param_struct_def Reference structure that is used to parse and match the parameters. The map values should be set to  
    */
    vector<ParameterPackageFetchStruct> fetchMatchingParametersFromList(ros::NodeHandle& nh, string name_of_node, string name_of_list, 
                                        ParameterPackageFetchStruct param_struct_def);
};
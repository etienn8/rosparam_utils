#include "rosparam_utils/xmlrpc_utils.hpp"

namespace xmlrpc_utils
{
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
    
    vector<ParameterPackageFetchStruct> fetchMatchingParametersFromList(ros::NodeHandle& nh, string name_of_node, string name_of_list, 
                                        ParameterPackageFetchStruct param_struct_def)
    {
        ROS_INFO_STREAM(name_of_node << "CONFIG: Automatically parsing parameter list named:" << name_of_list);

        XmlRpc::XmlRpcValue list_of_param_package;
        vector<ParameterPackageFetchStruct> output_struct_list;

        // Set all the flags indicating that the values are set to false;
        param_struct_def.resetTheSetFlags();

        if(nh.getParam(name_of_list, list_of_param_package))
        {
            if(list_of_param_package.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                for(int package_index = 0; package_index < list_of_param_package.size(); ++package_index)
                {
                    XmlRpc::XmlRpcValue param_package = list_of_param_package[package_index];

                    if(param_package.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                    {
                        auto param_package_iterator = param_package.begin();
                        
                        ParameterPackageFetchStruct new_param_struct = param_struct_def;

                        new_param_struct.param_package_name_ = param_package_iterator->first;
                        XmlRpc::XmlRpcValue& params = param_package_iterator->second;

                        if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
                        {
                            for(int index_param = 0; index_param < params.size(); ++index_param)
                            {
                                XmlRpc::XmlRpcValue parameter = params[index_param];
                                if(parameter.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                                {
                                    bool parameter_found = false;
                                    auto param_it = parameter.begin();

                                    const string& parameter_name = param_it->first;
                                    const XmlRpc::XmlRpcValue& parameter_value = param_it->second;

                                    if(parameter_value.getType() == XmlRpc::XmlRpcValue::TypeString)
                                    {
                                        auto param_hit_it = new_param_struct.string_params_.find(parameter_name);
                                        if (param_hit_it != new_param_struct.string_params_.end())
                                        {
                                            parameter_found = true;
                                            param_hit_it->second.first = static_cast<string>(parameter_value);
                                            // Indicate that the parameter value is set.
                                            param_hit_it->second.second = true;
                                        }
                                    }

                                    if(!parameter_found && (parameter_value.getType() == XmlRpc::XmlRpcValue::TypeDouble))
                                    {
                                        auto param_hit_it = new_param_struct.float_params_.find(parameter_name);
                                        if (param_hit_it != new_param_struct.float_params_.end())
                                        {
                                            parameter_found = true;
                                            param_hit_it->second.first = static_cast<float>(static_cast<double>(parameter_value));
                                            // Indicate that the parameter value is set.
                                            param_hit_it->second.second = true;
                                        }
                                    }

                                    if(!parameter_found && (parameter_value.getType() == XmlRpc::XmlRpcValue::TypeInt))
                                    {
                                        auto param_hit_it = new_param_struct.int_params_.find(parameter_name);
                                        if (param_hit_it != new_param_struct.int_params_.end())
                                        {
                                            parameter_found = true;
                                            param_hit_it->second.first = static_cast<int>(parameter_value);
                                            // Indicate that the parameter value is set.
                                            param_hit_it->second.second = true;
                                        }
                                    }

                                    if (!parameter_found)
                                    {
                                        ROS_WARN_STREAM("CONFIG: Unknown parameter or bad type in node " << name_of_node <<  ": " << parameter_name<< ". Will be ignored.");
                                    }
                                }
                                else
                                {
                                    ROS_WARN_STREAM("CONFIG: A parameter is either empty or not a struct.");
                                }
                            }
                        }

                        output_struct_list.push_back(std::move(new_param_struct));
                    }
                    else
                    {
                        ROS_ERROR_STREAM("CONFIG: A parameter package in the list is not a structure.");
                    }
                }
            }
            else
            {
                ROS_ERROR_STREAM("CONFIG: The parameter package list is not an array.");
            }
        }
        else
        {
                ROS_ERROR_STREAM("CONFIG: Can't find a parameter package list named: " << name_of_list);
        }
        
        return output_struct_list;
    }
}
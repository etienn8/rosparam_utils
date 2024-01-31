#pragma once

#include <string>
#include <map>
#include <utility>
#include <vector>

using std::map;
using std::pair;
using std::string;
using std::vector;

namespace xmlrpc_utils
{
    /**
     * @brief Data structure used to define all the possible parameters a specific list (package) of parameter could have. 
     * the name of the list is also stored. The maps used the expected name of the parameter as a key and use a pair as its value.
     * That pair contains the value of the parameter and a flag to indicate if the value was set.
    */
    class ParameterPackageFetchStruct
    {
        public:

            struct OptionsStruct
            {
                vector<string> string_parameter_name_list;
                vector<string> float_parameter_name_list;
                vector<string> int_parameter_name_list;
            };

            ParameterPackageFetchStruct() {};

            /**
             * @brief Construct based on given string lists of string and float parameters.
             * @param fetch_options Structure that contains the list of parameters to fetch for
             * the desired types.
            */
            ParameterPackageFetchStruct(OptionsStruct& fetch_options)
            {            
                // String parameters
                for (vector<string>::iterator string_param_name_it = fetch_options.string_parameter_name_list.begin(); string_param_name_it != fetch_options.string_parameter_name_list.end(); ++string_param_name_it)
                {
                    std::pair<string, bool> temp_pair("", false);
                    string_params_[*string_param_name_it] = temp_pair;
                }

                // Float parameters
                for (vector<string>::iterator float_param_name_it = fetch_options.float_parameter_name_list.begin(); float_param_name_it != fetch_options.float_parameter_name_list.end(); ++float_param_name_it)
                {
                    std::pair<float, bool> temp_pair(0.0f, false);
                    float_params_[*float_param_name_it] = temp_pair;
                }

                // Float parameters
                for (vector<string>::iterator int_param_name_it = fetch_options.int_parameter_name_list.begin(); int_param_name_it != fetch_options.int_parameter_name_list.end(); ++int_param_name_it)
                {
                    std::pair<int, bool> temp_pair(0., false);
                    int_params_[*int_param_name_it] = temp_pair;
                }
            }

            /**
             * @brief Name of the list of parameters.
            */
            string param_package_name_ = "";

            /**
             * @brief Map where the key is the name of the string parameter to fetch. The value of the map is a pair where
             * the first value is the value of the parameter and the second is a flag indicating if the value was set.
            */
            map<string, pair<string, bool>> string_params_;

            /**
             * @brief Map where the key is the name of the float or double parameter to fetch. The value of the map is 
             * a pair where the first value is the value of the parameter and the second is a flag indicating if the value 
             * was set.
            */
            map<string, pair<float, bool>> float_params_;

            /**
             * @brief Map where the key is the name of the int parameter to fetch. The value of the map is 
             * a pair where the first value is the value of the parameter and the second is a flag indicating if the value 
             * was set.
            */
            map<string, pair<int, bool>> int_params_;

            /**
             * @brief Set the flag in each element of all maps indicating that the value was set to false.  
            */
            void resetTheSetFlags()
            {
                for (auto string_param_it = string_params_.begin(); string_param_it != string_params_.end(); ++string_param_it)
                {
                    string_param_it->second.second = false;
                }

                for (auto float_param_it = float_params_.begin(); float_param_it != float_params_.end(); ++float_param_it)
                {
                    float_param_it->second.second = false;
                }

                for (auto int_param_it = int_params_.begin(); int_param_it != int_params_.end(); ++int_param_it)
                {
                    int_param_it->second.second = false;
                }
            }

    };
}
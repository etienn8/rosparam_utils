#pragma once

#include <string>
#include <map>
#include <utility>

using std::string;
using std::map;
using std::pair;

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
            /**
             * @brief Name of the list of parameters.
            */
            string param_pacakge_name_ = "";

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
            }

    };
}
#pragma once

#include <string_view>
#include <stdexcept>
#include <string>

namespace Errors {
    template<typename T>
    T* RequireNonNullParam(T* param, std::string_view paramName, std::string_view methodName) {
        if (param == nullptr) {
            throw std::invalid_argument(
                "Parameter"
                + static_cast<std::string>(paramName)
                + " in method"
                + static_cast<std::string>(methodName)
                + " was null when it"
                + " should not have been!  Check the stacktrace to find the responsible line of code - "
                + "usually, it is the first line of user-written code indicated in the stacktrace.  "
                + "Make sure all objects passed to the method in question were properly initialized -"
                + " note that this may not be obvious if it is being called under "
                + "dynamically-changing conditions!  Please do not seek additional technical assistance"
                + " without doing this first!");
        }
        return param;
    }
}

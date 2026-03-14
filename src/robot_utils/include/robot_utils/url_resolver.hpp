#ifndef ROBOT_UTILS__URL_RESOLVER_HPP_
#define ROBOT_UTILS__URL_RESOLVER_HPP_

#include <string>

namespace robot_utils {

class URLResolver {
public:
    static std::string getResolvedPath(const std::string& url);
};

} // namespace robot_utils

#endif // ROBOT_UTILS__URL_RESOLVER_HPP_
#include "robot_utils/url_resolver.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdexcept>

namespace robot_utils {

std::string URLResolver::getResolvedPath(const std::string& url) {
    if (url.find("package://") == 0) {
        size_t pkg_end = url.find('/', 10);
        if (pkg_end != std::string::npos) {
            std::string pkg_name = url.substr(10, pkg_end - 10);
            std::string relative_path = url.substr(pkg_end);
            return ament_index_cpp::get_package_share_directory(pkg_name) + relative_path;
        }
    }
    return url;
}

} // namespace robot_utils
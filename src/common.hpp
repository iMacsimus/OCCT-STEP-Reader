#include <map>
#include <filesystem>
#include <string>
#include <thread>
#include <format>
#include <ranges>
namespace rv = std::ranges::views;

#include "occt_headers.hpp"

void get_cl_args(
    int argc, const char **argv,
    std::map<std::string, bool> &is_specified,
    std::filesystem::path &file_path,
    std::filesystem::path &save_dir);
std::string help_message();

class MyProgressIndicator : public Message_ProgressIndicator {
public:
    MyProgressIndicator() {}
protected:
    virtual void Show(const Message_ProgressScope& theScope, 
                     const Standard_Boolean isForce) override {}
};

inline
void progress_bar(Message_ProgressIndicator &indicator, std::string message) {
  while((1.0 - indicator.GetPosition()) > 1e-4) {
    std::cout << std::format("\r{} {:10.5}%", message, indicator.GetPosition()*100) << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "\r" << message << " Done.                 " << std::endl;
}

template<typename... Ts>
void println(Ts... args) {
  std::cout << std::format(args...) << std::endl;
}

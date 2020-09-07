#include <iostream>
#include <utility>
using namespace std;

class Console {
protected:
    template <typename T>
    void log_argument(T t) {
        std::cout << t << ",";
    }
public:
    template <typename... Args>
    void log(Args&&... args) {
        int dummy[] = { 0, ((void) log_argument(std::forward<Args>(args)),0)... };
        cout << endl;
    }

    template <typename... Args>
    void warn(Args&&... args) {
        cout << "WARNING: ";
        int dummy[] = { 0, ((void) log_argument(std::forward<Args>(args)),0)... };
        cout << endl;
    }

    template <typename... Args>
    void error(Args&&... args) {
        cout << "ERROR: ";
        int dummy[] = { 0, ((void) log_argument(std::forward<Args>(args)),0)... };
        cout << endl;
    }
};
Console c;

#include <pybind11/embed.h>
#include <iostream>
#include <source_path.h>
#include <FunctionLogger.hpp>
namespace py = pybind11;

class PybindDemo
{
public:
    PybindDemo(){}
    ~PybindDemo(){}

    void Demo1(){
        LOG_FUNCTION;
        // import module
        LoadModule();

        int sum = myscript.attr("add")(3, 5).cast<int>();
        std::string greeting = myscript.attr("greet")("Alice").cast<std::string>();

        std::cout << "Sum: " << sum << std::endl;
        std::cout << greeting << std::endl;
    }

    void Demo2(){
        py::object data = myscript.attr("data");

        for (int i = 0; i < 10; i++) {
            double val = data.attr("__getitem__")("value").cast<double>();
            std::cout << "C++ got value = " << val << std::endl;
//            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

private:
    void LoadModule(){
        this->sys = py::module_::import("sys");
        sys.attr("path").attr("insert")(0, modulePath);
        this->myscript = py::module_::import("myscript");
    }

    // module
    py::module_ sys;
    py::module_ myscript;

    // path
    const std::string sourcePath = SOURCE_FILE_PATH;
    const std::string modulePath = sourcePath + "/Demo/PybindDemo/python";

};

int main() {
    py::scoped_interpreter guard{}; // 启动 Python 解释器

    auto pybindDemo = PybindDemo();

    pybindDemo.Demo1();
    pybindDemo.Demo2();
}

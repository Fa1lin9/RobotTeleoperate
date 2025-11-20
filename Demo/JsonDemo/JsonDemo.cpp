#include <JsonParser/JsonParser.hpp>
#include <source_path.h>

int main(int argc, char const *argv[])
{
    std::cout<<SOURCE_FILE_PATH<<std::endl;

    std::string filePath = static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/Ti5Robot.json";

    JsonParser jsonParser(filePath);

    json::object temp = jsonParser.GetJsonObject();

//    std::cout<<temp[""]<<std::endl;

    return 0;
}

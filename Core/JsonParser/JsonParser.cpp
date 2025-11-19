#include <JsonParser/JsonParser.hpp>

JsonParser::JsonParser(const std::string &filePath_)
    :filePath(filePath_)
{
    try {
        // read the file
        std::ifstream file(filePath);
        if(!file.is_open()){
            throw std::runtime_error("Failed to open : " + filePath);
        }
        std::ostringstream oss;
        oss << file.rdbuf();
        std::string jsonStr = oss.str();

        // parse the file
        boost::json::value jsonValue = boost::json::parse(jsonStr);
        boost::json::object root = jsonValue.as_object();

    }  catch (const boost::json::system_error& e) {
        std::cerr<<"Failed to parse the json:"<<e.what()<<std::endl;
    }  catch (const std::exception &e) {
        std::cerr<<"Error:"<<e.what()<<std::endl;
    }
}


JsonParser::~JsonParser(){

}

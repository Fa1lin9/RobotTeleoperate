#pragma once

#include <boost/json.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

namespace json = boost::json;

class JsonParser
{
public:
    JsonParser(const std::string &filePath);
    ~JsonParser();

private:

    std::string filePath;
    json::value rootValue;
    json::object rootObject;


};

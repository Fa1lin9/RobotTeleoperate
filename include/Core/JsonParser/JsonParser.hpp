#pragma once

#include <boost/json.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>

namespace json = boost::json;

class JsonParser
{
public:
    JsonParser(const std::string &filePath);
    ~JsonParser();

    static Eigen::MatrixXd JsonArray2EigenMatrixXd(const json::array &array);

    static Eigen::VectorXd JsonArray2EigenVectorXd(const json::array &array);

    inline json::value GetJsonValue(){
        return this->rootValue;};

    inline json::object GetJsonObject(){
        if(this->rootObject.empty()){
            std::cerr<<"[JsonParser::GetJsonObject] Get empty object! "<<std::endl;
        }
        return this->rootObject;};

private:

    std::string filePath;
    json::value rootValue;
    json::object rootObject;


};

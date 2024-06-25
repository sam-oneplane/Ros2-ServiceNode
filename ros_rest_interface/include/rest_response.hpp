#ifndef REST_RESPONSE_HPP
#define REST_RESPONSE_HPP

#include <json/json.h>
#include <crow.h>
#include <string>

class RestResponse {
public:
    RestResponse() = default;
    void get_response(std::string const &topic, crow::response &res, std::string const &msg) ;
    void post_response(crow::response &res);
    void error_msg(crow::response &res, int code, std::string &&err_msg) ;
    inline bool check_header_value(crow::request const &req) { 
        return (req.get_header_value("Content-Type") != "application/json");
    }
    bool is_paresed_from_stream(crow::request const &req);
    inline Json::Value get_json_request() {return json_request["data"];} 

private:
    Json::Value json_request;
};

void RestResponse::get_response(std::string const &topic, crow::response &res, std::string const &msg) {
    // create the JSON response
    Json::Value json_response;
    json_response["topic"] = topic;
    json_response["data"] = msg;
    Json::StreamWriterBuilder writer;
    std::string output = Json::writeString(writer, json_response);
    res.set_header("Content-Type", "application/json");
    res.write(output);  // send the response
    res.end();
}

void RestResponse::post_response(crow::response &res) {
    res.code = 200;
    res.write("Message published");
    res.end();
}

void RestResponse::error_msg(crow::response &res, int code, std::string &&err_msg) {
    res.code = code;
    res.write(err_msg);
    res.end();
}

bool RestResponse::is_paresed_from_stream(crow::request const &req) {

    Json::CharReaderBuilder reader;
    std::istringstream ss(req.body);
    std::string errs;
    bool is_parsed {(!Json::parseFromStream(reader, ss, &json_request, &errs))};
    bool is_data_member {(!json_request.isMember("data"))};
    return (is_parsed && is_data_member);
}

#endif
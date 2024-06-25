#ifndef ROS_REST_RELAY_HPP
#define ROS_REST_RELAY_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <unordered_map>
#include <memory>
#include <thread>
#include <mutex>
#include <sstream>
#include <functional>
#include "rest_response.hpp"

class RosRestRelay : public rclcpp::Node {
public:
    // RAII 
    RosRestRelay(const std::vector<std::string>& initial_topics) ;

    ~RosRestRelay() ; // Ensure the server thread is properly joined on destruction

private:

    void subscribe(const std::string& topic);

    void run_server();

    std::optional<geometry_msgs::msg::Point> data2point(const Json::Value &json_data);

    std::string generate_rest_get_massage(std::string const &topic);


    std::thread server_thread_; // Thread for the HTTP server
    std::mutex mutex_; // Mutex to protect shared resources
    std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Point>>> publishers_;
    std::unordered_map<std::string, std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Point>>> subscriptions_;
    std::unordered_map<std::string, geometry_msgs::msg::Point> last_messages_;
};


RosRestRelay::RosRestRelay(const std::vector<std::string>& initial_topics) : Node("ros_rest_relay") {
        // Subscribe to the initial list of topics
        for (const auto& topic : initial_topics) {
            subscribe(topic);
        }
        // Start the server in a separate thread
        server_thread_ = std::thread(std::bind(&RosRestRelay::run_server, this));
        
}

RosRestRelay::~RosRestRelay() {
    if (server_thread_.joinable()) {
        server_thread_.join();
    }// Ensure the server thread is properly joined on destruction
}

void RosRestRelay::subscribe(const std::string& topic) {
    subscriptions_[topic] = this->create_subscription<geometry_msgs::msg::Point>(
        topic, 10, [this, topic](geometry_msgs::msg::Point::SharedPtr msg) {
            last_messages_[topic] = *msg; 
        });
}


void RosRestRelay::run_server() {
    crow::SimpleApp app;
    
    CROW_ROUTE(app, "/<string>").methods(crow::HTTPMethod::GET)
    ([this](const crow::request &req, crow::response &res, std::string topic) {
        std::unique_lock<std::mutex> locker(mutex_, std::defer_lock);
        locker.lock();
        std::string msg {generate_rest_get_massage(topic)};
        locker.unlock();
        RestResponse rest_response;
        rest_response.get_response(topic, res, msg);
    });

    CROW_ROUTE(app, "/<string>").methods(crow::HTTPMethod::POST)
    ([this](crow::request &req, crow::response &res, std::string topic) {

        RestResponse rest_response;
        std::unique_lock<std::mutex> locker(mutex_, std::defer_lock);
        // Log incoming request
        RCLCPP_INFO(this->get_logger(), "Received POST request for topic: %s", topic.c_str());

        // Determine content type and parse accordingly
        if (rest_response.check_header_value(req)) {
            rest_response.error_msg(res, 415, std::move("Unsupported content type"));
            return;
        }
        if (rest_response.is_paresed_from_stream(req)) {
            rest_response.error_msg(res, 400, std::move("Invalid JSON"));
            return;
        }
        Json::Value json_data {rest_response.get_json_request()};
        
        auto geo_point = data2point(json_data);
        if (!geo_point.has_value()) {
            rest_response.error_msg(res, 400, std::move("Missing field in point"));
            return;
        }
        locker.lock();
        // iterate to find is publisher exists
        if (publishers_.find(topic) == publishers_.end()) {
            // if didn't find, create and add to publishers_
            publishers_[topic] = this->create_publisher<geometry_msgs::msg::Point>(topic, 10);
        }
        publishers_[topic]->publish(geo_point.value());
        locker.unlock();
        rest_response.post_response(res);
    });

    app.port(8080).multithreaded().run(); // Run the server
}

std::optional<geometry_msgs::msg::Point> RosRestRelay::data2point(const Json::Value &json_data) {
    if (!(json_data.isMember("x") && json_data.isMember("y") && json_data.isMember("z"))){
        return std::nullopt;
    }
    
    geometry_msgs::msg::Point point;
    point.x = json_data["x"].asFloat();
    point.y = json_data["y"].asFloat();
    point.z = json_data["z"].asFloat();
    return point;
}

std::string RosRestRelay::generate_rest_get_massage(std::string const &topic) {
    if (subscriptions_.find(topic) == subscriptions_.end()) {
        subscribe(topic);
        return std::string{"New Subscriber!"};
    }
    if(publishers_.find(topic) == publishers_.end())
        return std::string{"No massage published in this topic!"} ;

    std::ostringstream oss;
    geometry_msgs::msg::Point point {last_messages_[topic]};
    oss << "{" << "x: " <<  point.x << ", y: " << point.y << ", z: " << point.z << "}";
    
    return oss.str();
}

#endif
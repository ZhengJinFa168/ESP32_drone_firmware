#pragma once
#include <functional>
#include <map>
#include <vector>
#include <string>

using EventCallback = std::function<void(const void*)>;

class EventService {
public:
    static EventService& instance() {
        static EventService svc;
        return svc;
    }

    void subscribe(const std::string& event, EventCallback cb) {
        _listeners[event].push_back(cb);
    }

    void publish(const std::string& event, const void* data = nullptr) {
        for (auto& cb : _listeners[event]) cb(data);
    }

private:
    std::map<std::string, std::vector<EventCallback>> _listeners;
};

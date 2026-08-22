#ifndef WSTRING_H
#define WSTRING_H

#include <string>
#include <cstring>

class String {
private:
    std::string data;

public:
    String() : data("") {}
    String(const char* str) : data(str ? str : "") {}
    String(const std::string& str) : data(str) {}
    String(int value) : data(std::to_string(value)) {}
    String(unsigned int value) : data(std::to_string(value)) {}
    String(long value) : data(std::to_string(value)) {}
    String(float value) : data(std::to_string(value)) {}
    String(double value) : data(std::to_string(value)) {}

    const char* c_str() const { return data.c_str(); }
    unsigned int length() const { return data.length(); }
    bool isEmpty() const { return data.empty(); }

    String operator+(const String& rhs) const { return String(data + rhs.data); }
    String& operator+=(const String& rhs) { data += rhs.data; return *this; }

    bool operator==(const String& rhs) const { return data == rhs.data; }
    bool operator==(const char* rhs) const { return data == rhs; }
    bool operator!=(const String& rhs) const { return data != rhs.data; }

    char operator[](unsigned int index) const { return data[index]; }
};

#endif // WSTRING_H
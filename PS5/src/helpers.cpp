#include "helpers.hpp"

stringstream tokenize_string(const string &s, const string &delim) {
    stringstream ss;

    auto start = 0U;
    auto end = s.find(delim);

    while (end != string::npos) {
        ss << s.substr(start, end - start) << " ";
        start = end + delim.length();
        end = s.find(delim, start);
    }

    ss << s.substr(start, end);

    return ss;
}

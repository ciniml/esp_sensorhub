#ifndef STRING_UTILS_HPP__
#define STRING_UTILS_HPP__

#include <cstdint>
#include <cstring>
#include <string>

class ConstantString {
private:
	std::size_t _size;
	const char* _s;
public:
    template<std::size_t N>
    ConstantString(const char s[N]) : _s(s), _size(N) {}
	ConstantString(const char* s) : _s(s) {
		this->_size = std::strlen(s);
	}
    ConstantString(const char* s, std::size_t size) : _size(size), _s(s) {}
	ConstantString(const ConstantString& obj) : _size(obj._size), _s(obj._s) {}
	std::size_t size() const { return this->_size; }
	const char* c_str() const { return this->_s; }
};
static ConstantString operator"" _constant(const char* s, std::size_t size) {
    return ConstantString(s, size);
}

static std::size_t totalLength(const char* s) {
    return std::strlen(s);
}
static std::size_t totalLength(const ConstantString& s) {
	return s.size();
}
template<typename T, typename ... Types>
static std::size_t totalLength(const T& head, Types... tail) {
	return totalLength(head) + totalLength(tail...);
}
static void appendStrings(std::string& s, const char* ss) {
    s.append(ss);
}
static void appendStrings(std::string& s, const ConstantString& ss) {
	s.append(ss.c_str(), ss.size());
}
template<typename T, typename ... Types>
static void appendStrings(std::string& s, const T& head, Types... tail) {
	appendStrings(s, head);
	appendStrings(s, tail...);
}

template<typename ... Types>
static std::string concatStrings(Types... ss) {
	auto length = totalLength(ss...);
	std::string s;
	s.reserve(length);
	appendStrings(s, ss...);
	return s;
}

static const ConstantString a("hoge");
static const ConstantString b("fuga");
static const ConstantString c("piyopiyo");



#endif
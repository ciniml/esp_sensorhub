#ifndef LAZY_HPP__
#define LAZY_HPP__

#include <type_traits>

template<typename T>
class Lazy
{
private:
	typedef typename std::aligned_storage<sizeof(T), alignof(T)>::type storage_type;

	storage_type storage;
	T* initialized_target;

public:
	Lazy() : initialized_target(nullptr) {}

	void ensure()
	{
		if (this->initialized_target == nullptr) {
			this->initialized_target = new (reinterpret_cast<void*>(&this->storage)) T();
		}
	}

	T* get() {
		this->ensure();
		return this->initialized_target;
	}

	T* operator->() {
		return this->get();
	}
	operator T*() {
		return this->get();
	}
};

#endif //LAZY_HPP__

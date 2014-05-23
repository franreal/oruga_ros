
#ifndef BOOST_CRITICALSECTION_H
#define BOOST_CRITICALSECTION_H

#include <boost/thread/pthread/recursive_mutex.hpp>

template <typename T>
struct Critical {

	Critical(): read(true) {}

	~Critical() {}

	bool hasNewData() { return !read; }

	void get( T *value ) {

		boost::lock_guard<boost::recursive_mutex> lock(mutex);
		*value = data;
		read = true;
	}

	void set( const T *value ) {

		boost::lock_guard<boost::recursive_mutex> lock(mutex);
		data = *value;
		read = false;
	}

	void set( const T &value ) {

		boost::lock_guard<boost::recursive_mutex> lock(mutex);
		data = value;
		read = false;
	}

protected:
	T data;
	boost::recursive_mutex mutex;
	bool read;
};

#endif  // BOOST_CRITICALSECTION_H

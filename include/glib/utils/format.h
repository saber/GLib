#ifndef GLIB_FORMAT_H_
#define GLIB_FORMAT_H_

#include <string>
#include <cstdarg>

namespace glib {
using std::vector;

    //! \brief 格式化函数
    std::string format(const char *fmt, ...) {
    	if (!fmt) return std::string();
    	int result = -1, length = 2048;
    	std::string buffer;
    	while (result == -1)
    	{
    		buffer.resize(length);

    		va_list args;  // This must be done WITHIN the loop
    		va_start(args,fmt);
    		result =  vsnprintf(&buffer[0], length, fmt, args);
    		va_end(args);

    		// Truncated?
    		if (result>=length) result=-1;
    		length*=2;

    		// Ok?
    		if (result >= 0) {
    			buffer.resize(result);
    		}
    	}

    	return buffer;
    }

} // namespace kit

#endif

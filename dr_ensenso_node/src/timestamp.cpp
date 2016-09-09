#include "timestamp.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/time_facet.hpp>
#include <sstream>

namespace dr {

namespace {
	std::string formatTime(boost::posix_time::ptime timestamp, std::string const & format) {
		std::stringstream buffer;
		buffer.imbue(std::locale(buffer.getloc(), new boost::posix_time::time_facet(format.c_str())));
		buffer << timestamp;
		return buffer.str();
	}
}

std::string getTimeString(std::string const & format) {
	return  formatTime(boost::posix_time::microsec_clock::universal_time(), format);
}

std::string getTimeString() {
	return  getTimeString("%Y-%m-%d_%H-%M-%S.%f");
}

}

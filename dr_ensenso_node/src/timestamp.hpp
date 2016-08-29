#include <string>

namespace dr {

/// Get a string representing the time according to a specified format.
std::string getTimeString(
	std::string const & format ///< A boost::date_time compatible format string.
);

/// Get a string representing the time according to a fixed format.
/**
 * The exact format is implementation defined but contains both the date
 * and the time, with a precision of at least one millisecond.
 */
std::string getTimeString();

}

#pragma once
#include <ensenso/nxLib.h>

#include <string>
#include <stdexcept>


namespace dr {

/// Wrapper for NxLibException that also inherits from std::runtime_error.
class NxError : public NxLibException, public std::runtime_error {
public:
	NxError(std::string const & path, int error, std::string const & what = "");
	NxError(NxLibItem const & item,   int error, std::string const & what = "");
	NxError(NxLibException const & error,        std::string const & what = "");

	NxError(NxLibCommand const &, std::string const & path, int error, std::string const & what = "");
	NxError(NxLibCommand const &, NxLibItem const & item,   int error, std::string const & what = "");
	NxError(NxLibCommand const &, NxLibException const & error,        std::string const & what = "");
};

/// Execute an NxLibCommand.
/**
 * \throw NxError on failure.
 */
inline void executeNx(NxLibCommand const & command, std::string const & what = "") {
	int error = 0;
	command.execute(&error);
	if (error) throw NxError(command, itmExecute, error, what);
}

/// Set the value of an NxLibItem.
/**
 * \throw NxError on failure.
 */
template<typename T>
void setNx(NxLibItem const & item, T && value, std::string const & what = "") {
	int error = 0;
	item.set(&error, std::forward<T>(value));
	if (error) throw NxError(item, error, what);
}

/// Get the value of an NxLibItem as the specified type.
/**
 * \throw NxError on failure.
 */
template<typename T>
T getNx(NxLibItem const & item, std::string const & what = "") {
	int error = 0;
	T result = item.as<T>(&error);
	if (error) throw NxError(item, error, what);
	return result;
}

}

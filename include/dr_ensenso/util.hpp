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
inline void executeNx(NxLibCommand & command, std::string const & what = "") {
	try {
		command.execute();
	} catch (NxLibException const & e) {
		throw NxError(command, e, what);
	}
}

/// Execute an NxLibCommand.
/**
 * \throw NxError on failure.
 */
inline void executeNx(NxLibCommand && command, std::string const & what = "") {
	return executeNx(command, what);
}

/// Set the value of an NxLibItem.
/**
 * \throw NxError on failure.
 */
template<typename T>
void setNx(NxLibItem & item, T && value, std::string const & what = "") {
	try {
		item.set(std::forward<T>(value));
	} catch (NxLibException & e) {
		throw NxError(e, what);
	}
}

/// Set the value of an NxLibItem.
/**
 * \throw NxError on failure.
 */
template<typename T>
void setNx(NxLibItem && item, T && value, std::string const & what = "") {
	setNx(item, std::forward<T>(value), what);
}

/// Get the value of an NxLibItem as the specified type.
/**
 * \throw NxError on failure.
 */
template<typename T>
T getNx(NxLibItem const & item, std::string const & what = "") {
	try {
		return item.as<T>();
	} catch (NxLibException & e) {
		throw NxError(e, what);
	}
}

}

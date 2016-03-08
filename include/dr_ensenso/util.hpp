#pragma once
#include "error.hpp"

#include <ensenso/nxLib.h>

#include <string>
#include <stdexcept>


namespace dr {

/// Execute an NxLibCommand.
/**
 * \throw NxError on failure.
 */
void executeNx(NxLibCommand const & command, std::string const & what = "");

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

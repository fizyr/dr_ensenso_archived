#pragma once
#include "error.hpp"

#include <ensenso/nxLib.h>

#include <boost/optional.hpp>

#include <string>
#include <stdexcept>


namespace dr {

/// Find a camera by serial number.
boost::optional<NxLibItem> findCameraBySerial(std::string const & serial);

/// Find a camera by eeprom ID.
boost::optional<NxLibItem> findCameraByEepromId(int eeprom_id);

/// Find a camera that is linked to another camera given by serial.
boost::optional<NxLibItem> findCameraByLink(std::string const & serial);

/// Find and open a camera by serial number.
/**
 * \return The NxLibItem representing the found camera or an empty optional.
 * \throws if opening the camera fails.
 */
boost::optional<NxLibItem> openCameraBySerial(std::string const & serial);

/// Find and open a camera by eeprom ID.
/**
 * \return The NxLibItem representing the found camera or an empty optional.
 * \throws if opening the camera fails.
 */
boost::optional<NxLibItem> openCameraByEepromId(int eeprom_id);

/// Find and open a camera that is linked to another camera given by serial.
/**
 * \return The NxLibItem representing the found camera or an empty optional.
 * \throws if opening the camera fails.
 */
boost::optional<NxLibItem> openCameraByLink(std::string const & serial);

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

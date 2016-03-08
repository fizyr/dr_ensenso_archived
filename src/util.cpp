#include "util.hpp"

namespace dr {

boost::optional<NxLibItem> findCameraBySerial(std::string const & serial) {
	NxLibItem camera = NxLibItem{}[itmCameras][itmBySerialNo][serial];
	if (!camera.exists()) return {};
	return camera;
}

boost::optional<NxLibItem> findCameraByEepromId(int eeprom_id) {
	NxLibItem camera = NxLibItem{}[itmCameras][itmByEepromId][eeprom_id];
	if (!camera.exists()) return {};
	return camera;
}

boost::optional<NxLibItem> findCameraByLink(std::string const & serial) {
	NxLibItem cameras = NxLibItem{}[itmCameras][itmBySerialNo];
	for (int i = 0; i < cameras.count(); ++i) {
		if (getNx<std::string>(cameras[i][itmLink][itmTarget]) == serial) return cameras[i];
	}
	return {};
}

boost::optional<NxLibItem> findCameraByType(std::string const & type) {
	NxLibItem cameras = NxLibItem{}[itmCameras][itmBySerialNo];
	for (int i = 0; i < cameras.count(); ++i) {
		if (getNx<std::string>(cameras[i][itmType]) == type) return cameras[i];
	}
	return {};
}

namespace {
	/// Open an optional camera, or return nothing.
	boost::optional<NxLibItem> openCamera(boost::optional<NxLibItem> camera) {
		if (!camera) return {};

		NxLibCommand command(cmdOpen);
		setNx(command.parameters()[itmCameras], getNx<std::string>((*camera)[itmSerialNumber]));
		executeNx(command);

		return camera;
	}
}

boost::optional<NxLibItem> openCameraBySerial(std::string const & serial) {
	return openCamera(findCameraBySerial(serial));
}

boost::optional<NxLibItem> openCameraByEepromId(int eeprom_id) {
	return openCamera(findCameraByEepromId(eeprom_id));
}

boost::optional<NxLibItem> openCameraByLink(std::string const & serial) {
	return openCamera(findCameraByLink(serial));
}

boost::optional<NxLibItem> openCameraByType(std::string const & type) {
	return openCamera(findCameraByType(type));
}

void executeNx(NxLibCommand const & command, std::string const & what) {
	int error = 0;
	command.execute(&error);
	if (error) throwCommandError(error, what);
}

}

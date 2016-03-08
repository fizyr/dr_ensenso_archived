#include "ensenso.hpp"
#include "util.hpp"
#include "dump.hpp"

#include <dr_pcl/pointcloud_tools.hpp>
#include <dr_log/dr_log.hpp>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/time_facet.hpp>

#include <sstream>
#include <csignal>

namespace dr {

/// Tool for dumping live data from an Ensenso camera to files.
class EnsensoDumpTool {
	/// Ensenso camera wrapper.
	Ensenso ensenso_;

	/// Raw ensenso nxLib node.
	NxLibItem camera_;

	/// Optional raw overlay camera nxLib node.
	boost::optional<NxLibItem> overlay_;

	/// Flag to asynchronously stop the dump tool if it is running.
	volatile bool stop_;

	/// The directory to save the output files to.
	std::string output_directory;

public:
	struct {
		bool stereo_raw        = true;
		bool stereo_rectified  = true;
		bool disparity         = true;
		bool point_cloud       = true;
		bool overlay_raw       = true;
		bool overlay_rectified = true;
		bool calibration       = true;
		bool parameters        = true;
	} dump;

	/// Check the camera wrapper.
	Ensenso const & camera() const { return ensenso_; }

	/// Construct an ensenso dump tool.
	EnsensoDumpTool() {
		camera_  = ensenso_.native();
		overlay_ = ensenso_.nativeOverlay();
	}

	/// Destructor.
	~EnsensoDumpTool() {
		std::cerr << "Closing the camera. This may take some time.\n";
	}

	/// Load camera parameters for the main stereo camera.
	void loadCameraParameters(std::string const & filename) {
		setNxJsonFile(camera_[itmParameters], filename);
	}

	/// Load camera parameters for the linked overlay camera.
	void loadOverlayParameters(std::string const & filename) {
		if (!overlay_) throw std::runtime_error("No overlay camera linked. Can not set parameters.");
		setNxJsonFile(overlay_.get()[itmParameters], filename);
	}

	/// Set the connected cameras hardware or software triggered.
	void hardwareTriggered(bool enabled) {
		setNx(camera_[itmParameters][itmCapture][itmTriggerMode], enabled ? valFallingEdge : valSoftware);
		if (overlay_) {
			setNx((*overlay_)[itmParameters][itmCapture][itmTriggerMode], enabled ? valFallingEdge : valSoftware);
		}
	}

	/// Do a single trigger, retrieve, process, dump step.
	void step() {
		ensenso_.trigger();
		try {
			if (!ensenso_.retrieve(false)) return;
		} catch (NxCommandError const & e) {
			// Ignore timeouts, throw the rest of the errors.
			if (e.error_symbol() == errCaptureTimeout) return;
			throw;
		}

		// Compute disparity if needed.
		if (dump.disparity || dump.point_cloud) {
			NxLibCommand command(cmdComputeDisparityMap);
			setNx(command.parameters()[itmCameras], ensenso_.serialNumber());
			executeNx(command);
		}

		// Compute point cloud if needed.
		if (dump.point_cloud) {
			NxLibCommand command(cmdComputePointMap);
			setNx(command.parameters()[itmCameras], ensenso_.serialNumber());
			executeNx(command);
		}

		// Also get rectified overlay image.
		if (overlay_ && dump.overlay_rectified) {
			NxLibCommand command{cmdRectifyImages};
			setNx(command.parameters()[itmCameras], ensenso_.overlaySerialNumber());
			executeNx(command);
		}

		dumpData();
	}

	/// Continually run step() until stopped.
	void run() {
		stop_ = false;
		while (!stop_) step();
	}

	/// Stop the dump tool if it is currently running.
	void stop() {
		stop_ = true;
	}

protected:
	/// Dump all data
	void dumpData() {
		dumpCameraImages(
			camera_,
			output_directory + "/",
			"",
			default_time_format,
			dump.stereo_raw,
			dump.stereo_rectified,
			dump.disparity,
			dump.point_cloud
		);

		if (overlay_) dumpCameraImages(
			*overlay_,
			output_directory + "/",
			"",
			default_time_format,
			dump.overlay_raw,
			dump.overlay_rectified,
			false,
			false
		);
	}
};

}

namespace {
	dr::EnsensoDumpTool * dump_tool = nullptr;

	extern "C" void signal_handler(int) {
		if (dump_tool) dump_tool->stop();
	}
}

int main() {
	// Create and configure Ensenso
	std::cerr << "Initializing camera. This may take some time.\n";

	dr::EnsensoDumpTool dump_tool;
	::dump_tool = &dump_tool;
	std::signal(SIGINT, signal_handler);

	dump_tool.hardwareTriggered(true);

	std::cerr << "Camera initialized.\n";
	std::cerr << "Starting camera capture.\n";
	dump_tool.run();
	std::cerr << "Stopped image capture.\n";
}

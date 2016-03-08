#include "ensenso.hpp"
#include "util.hpp"
#include "opencv.hpp"

#include <dr_pcl/pointcloud_tools.hpp>
#include <dr_log/dr_log.hpp>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/time_facet.hpp>

#include <sstream>
#include <csignal>

namespace dr {


// Format a timestamp (microseconds since 1970) as a human readable string.
std::string formatTime(std::uint64_t timestamp) {
	boost::posix_time::ptime time(boost::gregorian::date(1970, 1, 1), boost::posix_time::microseconds(timestamp));

	std::stringstream buffer;
	buffer.imbue(std::locale(buffer.getloc(), new boost::posix_time::time_facet("%Y-%m-%d %H-%M-%S.%f")));
	buffer << time;
	return buffer.str();
}

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
		bool point_cloud       = true;
		bool disparity         = true;
		bool stereo_raw        = true;
		bool stereo_rectified  = true;
		bool overlay_raw       = true;
		bool overlay_rectified = true;
		bool calibration       = true;
		bool parameters        = true;
	} dump;

	/// Check the camera wrapper.
	Ensenso const & camera() const { return ensenso_; }

	/// Construct an ensenso dump tool.
	EnsensoDumpTool(bool software_trigger) {
		camera_  = ensenso_.getNativeCamera();
		overlay_ = ensenso_.getNativeOverlayCamera();

		setNx(camera_[itmParameters][itmCapture][itmTriggerMode], software_trigger ? valSoftware : valFallingEdge);
		if (overlay_) setNx((*overlay_)[itmParameters][itmCapture][itmTriggerMode], software_trigger ? valSoftware : valFallingEdge);
	}

	/// Destructor.
	~EnsensoDumpTool() {
		std::cerr << "Closing the camera. This may take some time.\n";
	}

	/// Do a single trigger, retrieve, dump step.
	void step() {
		ensenso_.trigger();
		try {
			if (!ensenso_.retrieve(false)) return;
		} catch (NxCommandError const & e) {
			// Ignore timeouts, throw the rest of the errors.
			if (e.error_symbol() == errCaptureTimeout) return;
			throw;
		}

		// Also get rectified overlay image.
		if (overlay_ && dump.overlay_rectified) {
			NxLibCommand command{cmdRectifyImages};
			setNx(command.parameters()[itmCameras], ensenso_.getOverlaySerialNumber());
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
	/// Dump a point cloud with a given description.
	void dumpCloud(pcl::PointCloud<pcl::PointXYZ> const & cloud, std::string const & description) {
		saveCloud(formatTime(cloud.header.stamp) + "_" + description + ".pcd", cloud);
	}

	/// Dump an image from a binary node with a given description.
	void dumpImage(NxLibItem const & item, std::string const & description, std::uint64_t timestamp) {
		NxLibCommand command(cmdSaveImage);
		setNx(command.parameters()[itmNode], item.path);
		setNx(command.parameters()[itmFilename], formatTime(timestamp) + "_" + description + ".png");
		executeNx(command);
	}

	/// Dump all stereo images from a node, possibly once for each flewview image.
	void dumpStereoImages(NxLibItem const & item, std::string const & description, std::uint64_t timestamp) {
		NxLibCommand command(cmdSaveImage);
		if (item.isObject()) {
			dumpImage(item[itmLeft], description + "_left", timestamp);
			dumpImage(item[itmRight], description + "_right", timestamp);
		} else {
			for (int i = 0; i < item.count(); ++i) {
				dumpImage(item[i][itmLeft], description + "_left_" + std::to_string(i), timestamp);
				dumpImage(item[i][itmRight], description + "_right" + std::to_string(i), timestamp);
			}
		}
	}

	/// Dump all data
	void dumpData() {
		pcl::PointCloud<pcl::PointXYZ> cloud = ensenso_.getPointCloud(cv::Rect(), false);
		std::uint64_t timestamp = cloud.header.stamp;
		std::cerr << "Retrieved data with timestamp " << formatTime(timestamp) << ".\n";
		if (dump.point_cloud) dumpCloud(cloud, "point_cloud");

		if (dump.disparity)        dumpImage(camera_[itmImages][itmDisparityMap], "disparity", timestamp);
		if (dump.stereo_raw)       dumpStereoImages(camera_[itmImages][itmRaw], "stereo_raw", timestamp);
		if (dump.stereo_rectified) dumpStereoImages(camera_[itmImages][itmRectified], "stereo_rectified", timestamp);

		if (overlay_ && dump.overlay_raw)       dumpImage((*overlay_)[itmImages][itmRaw],       "overlay_raw",       timestamp);
		if (overlay_ && dump.overlay_rectified) dumpImage((*overlay_)[itmImages][itmRectified], "overlay_rectified", timestamp);
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

	dr::EnsensoDumpTool dump_tool(false);
	::dump_tool = &dump_tool;
	std::signal(SIGINT, signal_handler);

	std::cerr << "Camera initialized.\n";
	std::cerr << "Starting camera capture.\n";
	dump_tool.run();
	std::cerr << "Stopped image capture.\n";
}

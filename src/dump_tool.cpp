#include "ensenso.hpp"
#include "util.hpp"
#include "opencv.hpp"

#include <dr_pcl/pointcloud_tools.hpp>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <sstream>
#include <csignal>

namespace dr {

using ptime = boost::posix_time::ptime;
using time_duration = boost::posix_time::time_duration;
using date  = boost::gregorian::date;

std::string formatTime(std::uint64_t timestamp) {
	ptime time = ptime(date(1970, 1, 1), boost::posix_time::microseconds(timestamp));

	std::stringstream buffer;
	buffer.imbue(std::locale(buffer.getloc(), new boost::posix_time::time_facet("%Y-%m-%d %H-%M-%S.%f")));
	buffer << time;
	return buffer.str();
}

class EnsensoDumpTool {
	dr::Ensenso ensenso_;
	NxLibItem camera_;
	boost::optional<NxLibItem> overlay_;
	bool stop_;

public:
	EnsensoDumpTool(bool software_trigger) {
		camera_  = ensenso_.getNativeCamera();
		overlay_ = ensenso_.getNativeOverlayCamera();

		camera_[itmParameters][itmCapture][itmTriggerMode] = software_trigger ? valSoftware : valFallingEdge;
		if (overlay_) (*overlay_)[itmParameters][itmCapture][itmTriggerMode] = software_trigger ? valSoftware : valFallingEdge;
	}

	~EnsensoDumpTool() {
		std::cerr << "Closing the camera. This may take some time.\n";
	}

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
		if (overlay_) {
			NxLibCommand command{cmdRectifyImages};
			setNx(command.parameters()[itmCameras], ensenso_.getOverlaySerialNumber());
			executeNx(command);
		}

		dumpData();
	}

	void run() {
		stop_ = false;
		while (!stop_) step();
	}

	void stop() {
		stop_ = true;
	}

protected:
	void dumpCloud(pcl::PointCloud<pcl::PointXYZ> const & cloud, std::string const & description) {
		saveCloud(formatTime(cloud.header.stamp) + "_" + description + ".pcd", cloud);
	}

	void dumpImage(NxLibItem const & item, std::string const & description, std::uint64_t timestamp) {
		NxLibCommand command(cmdSaveImage);
		setNx(command.parameters()[itmNode], item.path);
		setNx(command.parameters()[itmFilename], formatTime(timestamp) + "_" + description + ".png");
		executeNx(command);
	}

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

	void dumpData() {
		pcl::PointCloud<pcl::PointXYZ> cloud = ensenso_.getPointCloud(cv::Rect(), false);
		std::uint64_t timestamp = cloud.header.stamp;
		dumpCloud(cloud, "point_cloud");
		std::cerr << "Retrieved data with timestamp " << formatTime(timestamp) << ".\n";

		dumpImage(camera_[itmImages][itmDisparityMap], "disparity", timestamp);
		dumpStereoImages(camera_[itmImages][itmRaw], "stereo_raw", timestamp);
		dumpStereoImages(camera_[itmImages][itmRectified], "stereo_rectified", timestamp);

		if (overlay_) {
			dumpImage((*overlay_)[itmImages][itmRaw],       "overlay_raw",       timestamp);
			dumpImage((*overlay_)[itmImages][itmRectified], "overlay_rectified", timestamp);
		}
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

#include "ensenso.hpp"
#include "util.hpp"

namespace dr {

class EnsensoDumpTool {
	dr::Ensenso ensenso;
	NxLibItem camera;
	boost::optional<NxLibItem> overlay;
	bool software_trigger = false;

	int image_id = 1;

public:
	EnsensoDumpTool() {
		camera  = ensenso.getNativeCamera();
		overlay = ensenso.getNativeOverlayCamera();

		camera[itmParameters][itmCapture][itmTriggerMode] = valFallingEdge;
		if (overlay) (*overlay)[itmParameters][itmCapture][itmTriggerMode] = valFallingEdge;
	}

	void step() {
		if (!ensenso.retrieve(software_trigger)) {
			std::cerr << "Failed to retrieve image from ensenso.";
			return;
		}

		// a new image has been received and copied into the raw image node
		cv::Mat image;
		ensenso.loadIntensity(image);
		cv::imwrite("file:///home/delftrobotics/test.png", image);
	}

	void run() {
		while (true) step();
	}

};

}

int main() {
	// Create and configure Ensenso
	dr::EnsensoDumpTool dump_tool;
	dump_tool.run();
}

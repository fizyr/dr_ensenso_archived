#include "ensenso.hpp"
#include "util.hpp"

namespace dr {

class EnsensoDumpTool {
	dr::Ensenso ensenso;
	NxLibItem camera;
	boost::optional<NxLibItem> overlay;

	int image_id = 1;

public:
	EnsensoDumpTool() {
		camera = ensenso.getNativeCamera();
		overlay = ensenso.getNativeOverlayCamera();

		camera[itmParameters][itmCapture][itmTriggerMode] = valFallingEdge;
		if (overlay) (*overlay)[itmParameters][itmCapture][itmTriggerMode] = valFallingEdge;
	}

	void step() {
		NxLibCommand retrieve{cmdRetrieve};
		retrieve.parameters()[itmTimeout] = 0;
		executeNx(retrieve);
		bool retrieve_overlay = overlay && getNx<bool>(retrieve.result()[ensenso.getOverlaySerialNumber()][itmRetrieved]);
		bool retrieve_ensenso = getNx<bool>(retrieve.result()[ensenso.getSerialNumber()][itmRetrieved]);

		if (!retrieve_ensenso) {
			std::cerr << "Failed to retrieve image from ensenso.";
			return;
		}

		if (overlay && !retrieve_overlay) {
			std::cerr << "Failed to retrieve image from overlay camera.";
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

#include "dr_ensenso/ensenso.hpp"
#include "dr_ensenso/util.hpp"

namespace dr {

int main(int argc, char * argv[]) {
	// Create and configure Ensenso
	dr::Ensenso ensenso;
	ensenso.getNativeCamera()[itmParameters][itmCapture][itmTriggerMode] = valFallingEdge;
	boost::optional<NxLibItem> overlay = ensenso.getNativeOverlayCamera();
	if (overlay) (*overlay)[itmParameters][itmCapture][itmTriggerMode] = valFallingEdge;

	NxLibCommand retrieve{cmdRetrieve};
	while (true) {
		retrieve.parameters()[itmTimeout] = 0;
		executeNx(retrieve);
		bool retrieve_overlay = getNx<bool>(retrieve.result()[ensenso.getOverlaySerialNumber()][itmRetrieved]);
		bool retrieve_ensenso = getNx<bool>(retrieve.result()[ensenso.getSerialNumber()][itmRetrieved]);
		if (retrieve_ensenso && retrieve_overlay) {
			// a new image has been received and copied into the raw image node
			cv::Mat image;
			ensenso.loadIntensity(image);
			cv::imwrite("file:///home/delftrobotics/test.png", image);
			break;
		}
	}
}

}

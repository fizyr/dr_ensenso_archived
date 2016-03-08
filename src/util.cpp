#include "util.hpp"

namespace dr {

namespace {
	std::string makeErrorMsg(NxLibException const & error, std::string const & what) {
		return what + (what.empty() ? "" : ": ") + "NxLibException at " + error.getItemPath() + ": " + std::to_string(error.getErrorCode()) + ": " + error.getErrorText();
	}

	std::string makeErrorMsg(NxLibCommand const & command, NxLibException const & error, std::string const & what) {
		if (error.getErrorCode() == NxLibExecutionFailed) {
			std::string symbol = command.result()[itmErrorSymbol].asString();
			std::string message = command.result()[itmErrorText].asString();
			return what + (what.empty() ? "" : ": ") + "Failed to execute NxLibCommand: error " + symbol + ": " + message;
		}
		return makeErrorMsg(error, what);
	}
}

NxError::NxError(std::string const & path, int error, std::string const & what) : NxError({path,      error}, what) {}
NxError::NxError(NxLibItem const & item,   int error, std::string const & what) : NxError({item.path, error}, what) {}
NxError::NxError(NxLibException const & error, std::string const & what) : NxLibException(error), std::runtime_error(makeErrorMsg(error, what)) {}

NxError::NxError(NxLibCommand const & command, std::string const & path, int error, std::string const & what) : NxError(command, {path,      error}, what) {}
NxError::NxError(NxLibCommand const & command, NxLibItem const & item,   int error, std::string const & what) : NxError(command, {item.path, error}, what) {}
NxError::NxError(NxLibCommand const & command, NxLibException const & error, std::string const & what) : NxLibException(error), std::runtime_error(makeErrorMsg(command, error, what)) {}

}

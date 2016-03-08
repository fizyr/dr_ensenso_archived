#include "util.hpp"

namespace dr {

void executeNx(NxLibCommand const & command, std::string const & what) {
	int error = 0;
	command.execute(&error);
	if (error) throwCommandError(error, what);
}

}

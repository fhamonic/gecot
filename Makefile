BUILD_DIR = build
# CONAN_PROFILE = default_c++26
CONAN_PROFILE = debug_c++26

.PHONY: all test clean ${BUILD_DIR}

all: test package

$(BUILD_DIR):
	conan build . -of=${BUILD_DIR} -b=missing -pr=${CONAN_PROFILE} -s boost/*:compiler.cppstd=20

init-submodules:
	git submodule update --init --recursive
	
update-submodules:
	git submodule update --recursive --remote

test: $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
	ctest --output-on-failure

package: $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
	cpack

clean:
	@rm -rf CMakeUserPresets.json
	@rm -rf $(BUILD_DIR)

BUILD_DIR = build
BUILD_TYPE = default

.PHONY: all test clean ${BUILD_DIR}

all: test package

$(BUILD_DIR):
	conan build . -of=${BUILD_DIR} -b=missing -pr=${BUILD_TYPE}

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

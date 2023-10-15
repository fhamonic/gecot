BUILD_DIR = build

.PHONY: all test clean

all: test

$(BUILD_DIR):
	conan build . -of=${BUILD_DIR} -b=missing

init-submodules:
	git submodule update --init --recursive
	
update-submodules:
	git submodule update --recursive --remote

test: $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
	ctest --output-on-failure

clean:
	@rm -rf CMakeUserPresets.json
	@rm -rf $(BUILD_DIR)

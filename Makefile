BUILD_DIR = build
CONAN_PROFILE = gcc15_c++26
# CONAN_PROFILE = gcc15_c++26
# CONAN_PROFILE = gcc15_c++26_debug

.PHONY: all test clean format check-format ${BUILD_DIR}

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

format:
	find include src test -name "*.hpp" -o -name "*.cpp" | xargs clang-format -i

check-format:
	find include src test -name "*.hpp" -o -name "*.cpp" | xargs clang-format --dry-run -Werror

clean:
	@rm -rf CMakeUserPresets.json
	@rm -rf $(BUILD_DIR)

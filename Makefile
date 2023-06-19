MAKEFLAGS += --no-print-directory

CPUS?=$(shell getconf _NPROCESSORS_ONLN || echo 1)

CC = g++-12
BUILD_DIR = build

.PHONY: all clean doc

all: $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
	cmake --build . --parallel $(CPUS)

$(BUILD_DIR):
	@mkdir $(BUILD_DIR) && \
	cd $(BUILD_DIR) && \
	cmake -DCMAKE_CXX_COMPILER=$(CC) -DCMAKE_BUILD_TYPE=Release -DWARNINGS=ON -DOPTIMIZE_FOR_NATIVE=ON -DENABLE_TESTING=ON ..

test: all
	@cd $(BUILD_DIR) && \
	ctest --output-on-failure

test_preprocessing: all
	./build/test/landscape_opt_test --gtest_filter=preprocessing.fuzzy_test

# @cd $(BUILD_DIR) && \
# ctest -R preprocessing --output-on-failure

clean:
	@rm -rf $(BUILD_DIR)

doc:
	doxywizard $$PWD/docs/Doxyfile
	xdg-open docs/html/index.html

init-submodules:
	git submodule update --init --recursive
	
update-submodules:
	git submodule update --recursive --remote

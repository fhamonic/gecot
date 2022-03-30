MAKEFLAGS += --no-print-directory

CPUS?=$(shell getconf _NPROCESSORS_ONLN || echo 1)

BUILD_DIR = build

.PHONY: all clean doc

all: $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
	cmake --build . --parallel $(CPUS)

$(BUILD_DIR):
	@mkdir $(BUILD_DIR) && \
	cd $(BUILD_DIR) && \
	cmake -DCMAKE_BUILD_TYPE=Release -DWARNINGS=ON -DOPTIMIZE_FOR_NATIVE=OFF ..

test: all
	@cd $(BUILD_DIR) && \
	ctest --output-on-failure

clean:
	@rm -rf $(BUILD_DIR)

doc:
	doxywizard $$PWD/docs/Doxyfile
	xdg-open docs/html/index.html

update-submodules:
	git submodule update --recursive --remote

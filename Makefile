SHELL := /usr/bin/env bash
.DEFAULT_GOAL := help

ROS_SETUP ?= /opt/ros/jazzy/setup.bash
PKG ?= octa_ros
BUILD_TYPE ?= Debug
CMAKE_ARGS ?= -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
TEST_CMAKE_ARGS ?= -DENABLE_CLANG_TIDY=ON

COLCON = source "$(ROS_SETUP)" && colcon

CXX_FILES := $(shell git ls-files '*.c' '*.cc' '*.cpp' '*.cxx' '*.h' '*.hh' '*.hpp' '*.hxx')
TIDY_FILES := $(shell git ls-files 'src/*.c' 'src/*.cc' 'src/*.cpp' 'src/*.cxx')
MAKE_JOBS := $(shell echo '$(MAKEFLAGS)' | sed -n 's/.*-j\([0-9][0-9]*\).*/\1/p')
TIDY_JOBS ?= $(if $(MAKE_JOBS),$(MAKE_JOBS),$(shell (command -v nproc >/dev/null && nproc) || (command -v sysctl >/dev/null && sysctl -n hw.ncpu) || echo 4))
GCC_MAJOR ?= $(shell g++ -dumpversion | sed -E 's/^([0-9]+).*/\1/')
GCC_MULTIARCH ?= $(shell g++ -print-multiarch 2>/dev/null)

.PHONY: help build test format tidy lint clean

help:
	@echo "Make targets:"
	@echo "  build   - Build $(PKG) (BUILD_TYPE=$(BUILD_TYPE))"
	@echo "  test    - Run tests for $(PKG) and show results"
	@echo "  format  - clang-format all tracked C/C++ files (in-place)"
	@echo "  tidy    - run clang-tidy on tracked C/C++ files"
	@echo "  lint    - run format and clang-tidy together"
	@echo "  clean   - Remove build/install/log and compile_commands.json"

build:
	@set -eo pipefail; \
    if [ ! -f "$(ROS_SETUP)" ]; then \
      if [ -n "$$ROS_DISTRO" ] && [ -f "/opt/ros/$$ROS_DISTRO/setup.bash" ]; then \
        ROS_SETUP="/opt/ros/$$ROS_DISTRO/setup.bash"; \
        echo "Using ROS setup at $$ROS_SETUP (ROS_DISTRO=$$ROS_DISTRO)"; \
      else \
        echo "ROS setup not found at $(ROS_SETUP). Set ROS_SETUP or export ROS_DISTRO."; \
        exit 1; \
      fi; \
    fi; \
	$(COLCON) build --base-paths . --packages-select $(PKG) --cmake-args $(CMAKE_ARGS) -DENABLE_CLANG_TIDY=ON; \
	if [ -f build/$(PKG)/compile_commands.json ]; then \
	  ln -sf build/$(PKG)/compile_commands.json ./compile_commands.json; \
	elif [ -f build/compile_commands.json ]; then \
	  ln -sf build/compile_commands.json ./compile_commands.json; \
	fi

build-release:
	@$(MAKE) BUILD_TYPE=Release build

test:
	@set -eo pipefail; \
    if [ ! -f "$(ROS_SETUP)" ]; then \
      if [ -n "$$ROS_DISTRO" ] && [ -f "/opt/ros/$$ROS_DISTRO/setup.bash" ]; then \
        ROS_SETUP="/opt/ros/$$ROS_DISTRO/setup.bash"; \
        echo "Using ROS setup at $$ROS_SETUP (ROS_DISTRO=$$ROS_DISTRO)"; \
      else \
        echo "ROS setup not found at $(ROS_SETUP). Set ROS_SETUP or export ROS_DISTRO."; \
        exit 1; \
      fi; \
    fi; \
	$(COLCON) build --base-paths . --packages-select $(PKG) --cmake-args $(CMAKE_ARGS) $(TEST_CMAKE_ARGS); \
	$(COLCON) test --base-paths . --packages-select $(PKG) || true; \
	$(COLCON) test-result --verbose || true

.PHONY: tidy
tidy:
	@set -euo pipefail; \
	if ! command -v clang-tidy >/dev/null; then echo "clang-tidy not found"; exit 1; fi; \
	if [ ! -f compile_commands.json ]; then \
	  if [ -f build/$(PKG)/compile_commands.json ]; then ln -sf build/$(PKG)/compile_commands.json ./compile_commands.json; \
	  elif [ -f build/compile_commands.json ]; then ln -sf build/compile_commands.json ./compile_commands.json; \
	  else echo "compile_commands.json not found. Run 'make build' first."; exit 1; fi; \
	fi; \
	files="$(if $(FILE),$(FILE),$(TIDY_FILES))"; \
	if [ -z "$$files" ]; then echo "No C/C++ source files under src/."; exit 0; fi; \
	echo "Running clang-tidy (parallel: $(TIDY_JOBS)) on: $$(echo $$files | wc -w) files"; \
	printf '%s\n' $$files | xargs -r -n1 -P $(TIDY_JOBS) clang-tidy \
	  -p . \
	  --quiet \
	  --format-style=file \
	  -header-filter="^$(shell git rev-parse --show-toplevel 2>/dev/null || pwd)/src/" \
	  -extra-arg-before=-std=c++23 \
	  -extra-arg-before=-stdlib=libstdc++ \
	  -extra-arg-before=--gcc-toolchain=/usr \
	  -extra-arg-before=-isystem/usr/include/c++/$(GCC_MAJOR) \
	  -extra-arg-before=-isystem/usr/include/$(GCC_MULTIARCH)/c++/$(GCC_MAJOR)

format:
	@set -euo pipefail; \
	if ! command -v clang-format >/dev/null; then echo "clang-format not found"; exit 1; fi; \
	files="$(if $(FILE),$(FILE),$(CXX_FILES))"; \
	if [ -z "$$files" ]; then echo "No C/C++ source files found."; exit 0; fi; \
	echo "Formatting C/C++ sources... ($$(echo $$files | wc -w) files)"; \
	printf '%s\n' $$files | xargs -r clang-format -i

lint: format tidy

clean:
	@rm -rf build install log compile_commands.json

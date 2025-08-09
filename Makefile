SHELL := /usr/bin/env bash
.DEFAULT_GOAL := help

ROS_SETUP ?= /opt/ros/jazzy/setup.bash
PKG ?= octa_ros
BUILD_TYPE ?= Debug
CMAKE_ARGS ?= -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
TEST_CMAKE_ARGS ?= -DENABLE_CLANG_TIDY=ON

COLCON = source "$(ROS_SETUP)" && colcon

CXX_FILES := $(shell git ls-files '*.c' '*.cc' '*.cpp' '*.cxx' '*.h' '*.hh' '*.hpp' '*.hxx')
# Only source implementation files under src/ for clang-tidy (faster, avoids headers)
TIDY_FILES := $(shell git ls-files 'src/*.c' 'src/*.cc' 'src/*.cpp' 'src/*.cxx')
# Allow overriding number of jobs: make TIDY_JOBS=8 tidy
TIDY_JOBS ?= $(shell (command -v nproc >/dev/null && nproc) || (command -v sysctl >/dev/null && sysctl -n hw.ncpu) || echo 4)

.PHONY: help build test format format-check tidy lint clean

help:
	@echo "Make targets:"
	@echo "  build   - Build $(PKG) (BUILD_TYPE=$(BUILD_TYPE))"
	@echo "  test    - Run tests for $(PKG) and show results"
	@echo "  format  - clang-format all tracked C/C++ files (in-place)"
	@echo "  format-check - check formatting (no changes); nonzero exit on diff"
	@echo "  tidy    - run clang-tidy on tracked C/C++ files"
	@echo "  lint    - run format-check and clang-tidy together"
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
	$(COLCON) build --base-paths . --packages-select $(PKG) --cmake-args $(CMAKE_ARGS); \
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
	# Build with clang-tidy enabled (C/C++ only) before running tests
	$(COLCON) build --base-paths . --packages-select $(PKG) --cmake-args $(CMAKE_ARGS) $(TEST_CMAKE_ARGS); \
	# Run tests
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
	printf '%s\n' $$files | xargs -r -n1 -P $(TIDY_JOBS) tools/clang-tidy-wrapper.sh --project-root "$(shell git rev-parse --show-toplevel 2>/dev/null || pwd)" --allow-subdir src -p . --format-style=file

.PHONY: tidy-changed tidy-staged
tidy-changed:
	@set -euo pipefail; \
	base=$$(git merge-base HEAD $${BASE_REF:-origin/main} 2>/dev/null || echo HEAD~1); \
	files=$$(git diff --name-only $$base -- 'src/*.c' 'src/*.cc' 'src/*.cpp' 'src/*.cxx' | tr '\n' ' '); \
	if [ -z "$$files" ]; then echo "No changed C/C++ sources under src/."; exit 0; fi; \
	$(MAKE) FILE="$$files" tidy

tidy-staged:
	@set -euo pipefail; \
	files=$$(git diff --cached --name-only -- 'src/*.c' 'src/*.cc' 'src/*.cpp' 'src/*.cxx' | tr '\n' ' '); \
	if [ -z "$$files" ]; then echo "No staged C/C++ sources under src/."; exit 0; fi; \
	$(MAKE) FILE="$$files" tidy

format:
	@set -euo pipefail; \
	if ! command -v clang-format >/dev/null; then echo "clang-format not found"; exit 1; fi; \
	if [ -n "$(CXX_FILES)" ]; then \
	  echo "Formatting C/C++ sources..."; \
	  printf '%s\n' $(CXX_FILES) | xargs -r clang-format -i; \
	else \
	  echo "No C/C++ source files found."; \
	fi

format-check:
	@set -euo pipefail; \
	if ! command -v clang-format >/dev/null; then echo "clang-format not found"; exit 1; fi; \
	if [ -n "$(CXX_FILES)" ]; then \
	  echo "Checking formatting..."; \
	  printf '%s\n' $(CXX_FILES) | xargs -r -n1 clang-format -n --Werror; \
	else \
	  echo "No C/C++ source files found."; \
	fi

lint: format-check tidy

clean:
	@rm -rf build install log compile_commands.json

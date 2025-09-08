SHELL := /usr/bin/env bash
.DEFAULT_GOAL := help

ROS_SETUP ?= /opt/ros/jazzy/setup.bash
PKG ?= octa_ros
BUILD_TYPE ?= Debug
CMAKE_ARGS ?= -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
TEST_CMAKE_ARGS ?= -DENABLE_CLANG_TIDY=ON
BUILD_BASE ?= build
INSTALL_BASE ?= install
LOG_BASE ?= log

COLCON = source "$(ROS_SETUP)" && colcon

CXX_FILES := $(shell git ls-files '*.c' '*.cc' '*.cpp' '*.cxx' '*.h' '*.hh' '*.hpp' '*.hxx')
TIDY_FILES := $(shell git ls-files 'src/*.c' 'src/*.cc' 'src/*.cpp' 'src/*.cxx')
PY_FILES := $(shell git ls-files '*.py')
MAKE_JOBS := $(shell echo '$(MAKEFLAGS)' | sed -n 's/.*-j\([0-9][0-9]*\).*/\1/p')
TIDY_JOBS ?= $(if $(MAKE_JOBS),$(MAKE_JOBS),$(shell (command -v nproc >/dev/null && nproc) || (command -v sysctl >/dev/null && sysctl -n hw.ncpu) || echo 4))
GCC_MAJOR ?= $(shell g++ -dumpversion | sed -E 's/^([0-9]+).*/\1/')
GCC_MULTIARCH ?= $(shell g++ -print-multiarch 2>/dev/null)

.PHONY: help build test format tidy lint clean docker-ci-test dev run

help:
	@echo "Make targets:"
	@echo "  build   - Build $(PKG) (BUILD_TYPE=$(BUILD_TYPE))"
	@echo "  test    - Run tests for $(PKG) and show results"
	@echo "  format  - clang-format all tracked C/C++ files (in-place)"
	@echo "  tidy    - run clang-tidy on tracked C/C++ files"
	@echo "  lint    - run format and clang-tidy together"
	@echo "  clean   - Remove build/install/log and compile_commands.json"
	@echo "  run     - Run deploy container"
	@echo "  dev     - Run dev container"
	@echo "  docker-ci-test  - Build image if needed and run tests in it"

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
	rm -rf $(BUILD_BASE)/$(PKG)/Testing $(BUILD_BASE)/$(PKG)/test_results $(BUILD_BASE)/Testing $(BUILD_BASE)/test_results || true; \
	$(COLCON) build --base-paths . --packages-select $(PKG) \
	--build-base $(BUILD_BASE) --install-base $(INSTALL_BASE) \
	--cmake-args $(CMAKE_ARGS) $(TEST_CMAKE_ARGS) \
	--event-handlers console_cohesion+; \
	$(COLCON) test --base-paths . --packages-select $(PKG) \
	--build-base $(BUILD_BASE) --install-base $(INSTALL_BASE) \
	--event-handlers console_cohesion+ \
	--ctest-args -j 1 -VV; \
	$(COLCON) test-result --verbose --test-result-base $(BUILD_BASE)

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
	files_cxx="$(if $(FILE),$(FILE),$(CXX_FILES))"; \
	if [ -n "$$files_cxx" ]; then \
	  echo "Formatting C/C++ sources... ($$(echo $$files_cxx | wc -w) files)"; \
	  printf '%s\n' $$files_cxx | xargs -r clang-format -i; \
	else \
	  echo "No C/C++ source files found."; \
	fi; \
	if command -v ruff >/dev/null; then \
	  files_py="$(PY_FILES)"; \
	  if [ -n "$$files_py" ]; then \
	    echo "Running ruff (format + lint --fix) on Python files..."; \
	    ruff format $$files_py; \
	    ruff check --fix $$files_py; \
	  else \
	    echo "No Python files found."; \
	  fi; \
	else \
	  echo "ruff not found."; \
	fi

lint: tidy format

clean:
	@rm -rf build install log compile_commands.json

DOCKER_CI_TAG ?= octa_ros-ci:local
DOCKERFILE ?= docker/Dockerfile

docker-ci-test:
	@set -e; \
	docker buildx build --load -t $(DOCKER_CI_TAG) -f $(DOCKERFILE) --no-cache .; \
	docker run --rm --name octa_ci \
	  -u "$(shell id -u):$(shell id -g)" \
	  -e HOME=/tmp \
	  -w /workspace/repo \
	  -v "$(PWD):/workspace/repo" \
	  $(DOCKER_CI_TAG) \
	  bash -lc 'set -euo pipefail; set +u; source /opt/ros/jazzy/setup.bash; set -u; \
	    make -C /workspace/repo BUILD_BASE=/tmp/colcon_build INSTALL_BASE=/tmp/colcon_install LOG_BASE=/tmp/colcon_log test'

.PHONY: dev
dev:
	@set -e; \
	echo "ROBOT_IP=$(ROBOT_IP)"; \
	ROBOT_IP="$(ROBOT_IP)" docker compose -f docker/docker-compose-dev.yaml up --build

.PHONY: run
run:
	@set -e; \
	echo "ROBOT_IP=$(ROBOT_IP)"; \
	ROBOT_IP="$(ROBOT_IP)" docker compose -f docker/docker-compose.yaml up

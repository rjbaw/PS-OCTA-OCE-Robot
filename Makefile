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

.PHONY: build clean dev dev-cpu down format help lint local run test tidy test-ci logs status shell restart prune-logs

help:
	@echo "Make targets:"
	@echo "  build  - Build $(PKG) (BUILD_TYPE=$(BUILD_TYPE))"
	@echo "  test   - Run tests for $(PKG) and show results"
	@echo "  test-ci - Run tests excluding test_bag.py"
	@echo "  format - clang-format tracked C/C++ files; ruff on Python"
	@echo "  tidy   - Run clang-tidy on tracked C/C++ files"
	@echo "  lint   - Run 'tidy' and 'format'"
	@echo "  clean  - Remove build/install/log and compile_commands.json"
	@echo "  run    - Start deploy container"
	@echo "  dev    - Start dev container (mounts source, bash)"
	@echo "  dev-cpu - Start cpu dev container (mounts source, bash)"
	@echo "  local  - Build deployment image from local sources"
	@echo "  down   - Stop both dev and run containers"
	@echo "  logs   - Prune old logs, then show last 300 lines"
	@echo "  status - Show container, manager, and robot reachability"
	@echo "  shell  - Open an interactive shell in the container"
	@echo "  restart - Restart container (down then run)"

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
	rm -rf \
	  $(BUILD_BASE)/$(PKG)/Testing \
	  $(BUILD_BASE)/$(PKG)/test_results \
	  $(BUILD_BASE)/Testing \
	  $(BUILD_BASE)/test_results \
	  $(BUILD_BASE)/$(PKG)/CMakeCache.txt || true; \
	$(COLCON) build --base-paths . --packages-select $(PKG) \
	--build-base $(BUILD_BASE) --install-base $(INSTALL_BASE) \
	--cmake-args $(CMAKE_ARGS) $(TEST_CMAKE_ARGS) \
	--event-handlers console_cohesion+; \
	$(COLCON) test --base-paths . --packages-select $(PKG) \
	--build-base $(BUILD_BASE) --install-base $(INSTALL_BASE) \
	--event-handlers console_cohesion+ \
	--ctest-args -j 1 -VV; \
	$(COLCON) test-result --verbose --test-result-base $(BUILD_BASE)

test-ci:
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
	rm -rf \
	  $(BUILD_BASE)/$(PKG)/Testing \
	  $(BUILD_BASE)/$(PKG)/test_results \
	  $(BUILD_BASE)/Testing \
	  $(BUILD_BASE)/test_results \
	  $(BUILD_BASE)/$(PKG)/CMakeCache.txt || true; \
	$(COLCON) build --base-paths . --packages-select $(PKG) \
	--build-base $(BUILD_BASE) --install-base $(INSTALL_BASE) \
	--cmake-args $(CMAKE_ARGS) $(TEST_CMAKE_ARGS) \
	--event-handlers console_cohesion+; \
	$(COLCON) test --base-paths . --packages-select $(PKG) \
	--build-base $(BUILD_BASE) --install-base $(INSTALL_BASE) \
	--event-handlers console_cohesion+ \
	--ctest-args -j 1 -VV -E test_bag; \
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

DOCKER_TAG ?= ghcr.io/rjbaw/octa_ros:latest
DOCKERFILE ?= docker/Dockerfile

local:
	@set -e; \
	echo "Building local deployment image: $(DOCKER_TAG)"; \
	docker buildx build --load -t $(DOCKER_TAG) -f $(DOCKERFILE) --target run .

.PHONY: dev
dev:
	@set -e; \
	echo "ROBOT_IP=$(ROBOT_IP)"; \
	mkdir -p logs result; \
	UID=$$(id -u) GID=$$(id -g) ROBOT_IP="$(ROBOT_IP)" docker compose -f docker/docker-compose-dev.yaml up -d

.PHONY: dev-cpu
dev-cpu:
	@set -e; \
	echo "ROBOT_IP=$(ROBOT_IP)"; \
	mkdir -p logs result; \
	UID=$$(id -u) GID=$$(id -g) ROBOT_IP="$(ROBOT_IP)" docker compose -f docker/docker-compose-cpu.yaml up -d

.PHONY: run
run:
	@set -e; \
	echo "ROBOT_IP=$(ROBOT_IP)"; \
	mkdir -p logs result; \
	UID=$$(id -u) GID=$$(id -g) ROBOT_IP="$(ROBOT_IP)" docker compose -f docker/docker-compose.yaml up -d

.PHONY: down
down:
	@set -e; \
	docker compose -f docker/docker-compose.yaml down --remove-orphans || true; \
	docker compose -f docker/docker-compose-dev.yaml down --remove-orphans || true; \
	docker compose -f docker/docker-compose-cpu.yaml down --remove-orphans || true

logs:
	@set -euo pipefail; \
	PRUNE_LOGS_DAYS=$(PRUNE_LOGS_DAYS) $(MAKE) -s prune-logs; \
	CRASH=$$(ls -1t logs/ros_crash_*.log 2>/dev/null | head -n1 || true); \
	if [ -n "$$CRASH" ]; then \
	  echo "Last crash log: $$CRASH"; \
	  sed -e 's/^/[ROS] /' "$$CRASH"; \
	  exit 0; \
	fi; \
	LATEST_DIR=$$(find logs -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n1 | cut -d' ' -f2-); \
	if [ -n "$$LATEST_DIR" ]; then \
	  echo "RCUTILS logs in: $$LATEST_DIR"; \
	  find "$$LATEST_DIR" -type f -name '*.log' -print0 | xargs -0 -r tail -n 300; \
	  exit 0; \
	fi; \
	echo "No host logs found; checking in-container logs..."; \
	docker exec ps-oce-robot bash -lc 'LOGDIR=$${RCUTILS_LOGGING_DIRECTORY:-$$HOME/.ros/log}; echo "Container log dir: $$LOGDIR"; if [ -d "$$LOGDIR" ]; then find "$$LOGDIR" -type f -name "*.log" -print0 | xargs -0 -r tail -n 300; else echo "No container logs available."; fi' || true

status:
	@set -e; \
	echo "Container status:"; \
	docker ps -a --filter name=ps-oce-robot --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'; \
	echo; \
	echo "ROBOT_IP=$(ROBOT_IP)"; \
	if [ -n "$(ROBOT_IP)" ]; then \
	  if ping -c 1 -W 1 $(ROBOT_IP) >/dev/null 2>&1; then echo "Robot: reachable"; else echo "Robot: unreachable"; fi; \
	else echo "Robot: (ROBOT_IP not set)"; fi; \
	echo; \
	if docker ps --filter name=ps-oce-robot --format '{{.Names}}' | grep -qx ps-oce-robot; then \
	  echo "Inside container:"; \
	  docker exec ps-oce-robot bash -lc 'if pgrep -f "driver_manager.py" >/dev/null; then echo manager: running; else echo manager: not running; fi; if pgrep -f "ros2 launch octa_ros launch.py" >/dev/null; then echo driver: running; else echo driver: not running; fi; echo -n "Log dir: "; echo $${RCUTILS_LOGGING_DIRECTORY:-$$HOME/.ros/log}; last=$$(ls -1t logs/ros_crash_*.log 2>/dev/null | head -n1 || true); if [ -n "$$last" ]; then echo "Last crash log: $$last"; tail -n 5 "$$last" | sed -e "s/^/[ROS] /"; fi'; \
	else \
	  echo "Container not running; skipping inner status."; \
	fi

.PHONY: shell
shell:
	@set -e; \
	if docker ps --filter name=ps-oce-robot --format '{{.Names}}' | grep -qx ps-oce-robot; then \
	  docker exec -it ps-oce-robot bash; \
	else \
	  echo "Container 'ps-oce-robot' is not running. Start it with 'make run'."; exit 1; \
	fi

.PHONY: restart
restart:
	@set -e; \
	$(MAKE) down; \
	$(MAKE) run

.PHONY: prune-logs
PRUNE_LOGS_DAYS ?= 7
prune-logs:
	@set -e; \
	mkdir -p logs; \
	find logs -mindepth 1 -mtime +$(PRUNE_LOGS_DAYS) -exec rm -rf {} + 2>/dev/null || true; \
	echo "Pruned logs older than $(PRUNE_LOGS_DAYS) days from ./logs"

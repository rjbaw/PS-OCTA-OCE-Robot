#!/usr/bin/env bash
# Wrapper to run clang-tidy only on project sources and never fail the build.
# Recognized wrapper-only flags:
#   --project-root <abs_path>
#   --allow-subdir <name>   # can be provided multiple times (e.g., src, include)

set -euo pipefail

# Select clang-tidy executable
if command -v clang-tidy >/dev/null 2>&1; then
  CT=clang-tidy
elif command -v clang-tidy-18 >/dev/null 2>&1; then
  CT=clang-tidy-18
elif command -v clang-tidy-17 >/dev/null 2>&1; then
  CT=clang-tidy-17
elif command -v clang-tidy-16 >/dev/null 2>&1; then
  CT=clang-tidy-16
else
  echo "clang-tidy not found; skipping lint invocation." 1>&2
  exit 0
fi

project_root=""
allow_subdirs=()
# Cache GCC major version to avoid repeated subprocess calls
GCC_MAJOR=""
if command -v g++ >/dev/null 2>&1; then
  ver=$(g++ -dumpversion 2>/dev/null || true)
  GCC_MAJOR=${ver%%.*}
fi

# Parse wrapper flags and forward the rest
forward=( )
while [[ $# -gt 0 ]]; do
  case "$1" in
    --project-root)
      project_root="$2"; shift 2;;
    --allow-subdir)
      allow_subdirs+=("$2"); shift 2;;
    *)
      forward+=("$1"); shift;;
  esac
done

# Find the main source file from arguments (first path that looks like a source)
src_file=""
for a in "${forward[@]}"; do
  case "$a" in
    *.c|*.cc|*.cpp|*.cxx|*.h|*.hh|*.hpp|*.hxx)
      src_file="$a"; break;;
  esac
done

# Normalize paths
if [[ -n "$src_file" ]]; then
  if command -v realpath >/dev/null 2>&1; then
    src_file_abs=$(realpath -m "$src_file")
  else
    src_file_abs=$(readlink -f "$src_file" 2>/dev/null || echo "$src_file")
  fi
else
  # No identifiable source file; run but do not fail
  "${CT}" "${forward[@]}" || true
  exit 0
fi

# Default allow list if none provided: only src
if [[ ${#allow_subdirs[@]} -eq 0 ]]; then
  allow_subdirs=(src)
fi

# If project_root is not provided, try to derive it from git; else use cwd
if [[ -z "$project_root" ]]; then
  if command -v git >/dev/null 2>&1 && git rev-parse --show-toplevel >/dev/null 2>&1; then
    project_root=$(git rev-parse --show-toplevel)
  else
    project_root=$(pwd)
  fi
fi

# Ensure project_root is absolute
if command -v realpath >/dev/null 2>&1; then
  project_root=$(realpath -m "$project_root")
fi

# Decide whether to run clang-tidy on this source (limit to project sources)
run_tidy=false
case "$src_file_abs" in
  "$project_root"/*)
    # Check allow subdirs
    for sd in "${allow_subdirs[@]}"; do
      case "$src_file_abs" in
        "$project_root/$sd"/*) run_tidy=true; break;;
      esac
    done
    ;;
  *)
    run_tidy=false;;
esac

# Prevent diagnostics from external headers flooding output by passing a strict header-filter for this file
if [[ "$run_tidy" == true ]]; then
  # Derive a header-filter anchored to project_root
  header_filter="^${project_root}/src/"
  # Reduce noise and suppress clang parser diagnostics; keep clang-tidy checks
  # Force compile flags to include C++ stdlib and suppress parser diagnostics
  forward=(
    "-quiet"
    "-extra-arg=-Wno-everything"
    "-extra-arg=-fmacro-backtrace-limit=0"
    "-extra-arg=-fno-caret-diagnostics"
    "-extra-arg-before=-std=c++23"
    "-extra-arg-before=-stdlib=libstdc++"
    ${GCC_MAJOR:+"-extra-arg-before=-isystem/usr/include/c++/${GCC_MAJOR}"}
    ${GCC_MAJOR:+"-extra-arg-before=-isystem/usr/include/x86_64-linux-gnu/c++/${GCC_MAJOR}"}
    "-header-filter=${header_filter}"
    "${forward[@]}"
  )
fi

if [[ "$run_tidy" != true ]]; then
  # Skip linting for files outside allowed project source paths
  exit 0
fi

"${CT}" "${forward[@]}" || true

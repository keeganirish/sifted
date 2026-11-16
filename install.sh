#!/usr/bin/env bash
# Sifted installer for Linux, macOS and Windows (inside WSL; on Windows run
# install.cmd, which calls this). Checks and installs what the build needs,
# builds the fusion service, checks the result and adds a shortcut.
#
#   ./install.sh                  install (asks before installing system packages)
#   ./install.sh --yes            don't ask
#   ./install.sh --with-tests     also build and run the test suites
#   ./install.sh --help           all options
#
# Safe to run again: it only installs what is missing, rebuilds incrementally
# and overwrites its own shortcut. Full output goes to build/install.log.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PRESET=app
BUILD_DIR="$ROOT/build/$PRESET"
LOG="$ROOT/build/install.log"

ASSUME_YES=0 WITH_TESTS=0 SHORTCUT=1 SYSTEM_PKGS=1 CLEAN=0 FROM_WINDOWS=0
usage() {
  cat <<'EOF'
Usage: ./install.sh [options]

  -y, --yes               install missing system packages without asking
      --with-tests        also build and run the C++ and Python test suites
      --no-shortcut       don't create a desktop / application-menu shortcut
      --no-system-packages
                          never use the system package manager; download Eigen
                          (and GoogleTest) at build time instead (needs git)
      --clean             delete the previous build first
  -h, --help              show this help

Environment:
  CXX                     C++ compiler to use (default: the first one that works)
  SIFTED_CMAKE_ARGS       extra arguments for the CMake configure step
EOF
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    -y|--yes) ASSUME_YES=1 ;;
    --with-tests) WITH_TESTS=1 ;;
    --no-shortcut) SHORTCUT=0 ;;
    --no-system-packages) SYSTEM_PKGS=0 ;;
    --clean) CLEAN=1 ;;
    --from-windows) FROM_WINDOWS=1; SHORTCUT=0 ;;  # install.cmd makes the Windows shortcut
    -h|--help) usage; exit 0 ;;
    *) echo "install.sh: unknown option '$1' (see --help)" >&2; exit 2 ;;
  esac
  shift
done

# ----------------------------------------------------------------------------- output
if [[ -t 1 ]]; then
  B=$'\033[1m' DIM=$'\033[2m' RED=$'\033[31m' GRN=$'\033[32m' YEL=$'\033[33m' BLU=$'\033[34m' RST=$'\033[0m'
else
  B="" DIM="" RED="" GRN="" YEL="" BLU="" RST=""
fi
STEP=0 STEPS=6
step() { STEP=$((STEP + 1)); printf '\n%s[%d/%d] %s%s\n' "$B$BLU" "$STEP" "$STEPS" "$1" "$RST"; echo "=== $1" >>"$LOG"; }
ok()   { printf '  %s✓%s %s\n' "$GRN" "$RST" "$1"; }
info() { printf '  %s•%s %s\n' "$DIM" "$RST" "$1"; }
warn() { printf '  %s!%s %s\n' "$YEL" "$RST" "$1"; }
die() {
  printf '\n%s✗ %s%s\n' "$RED$B" "$1" "$RST" >&2
  [[ $# -gt 1 ]] && printf '  %s\n' "${@:2}" >&2
  if [[ -s "$LOG" ]]; then
    printf '\n%sLast lines of %s:%s\n' "$DIM" "$LOG" "$RST" >&2
    tail -n 25 "$LOG" | sed 's/^/    /' >&2
  fi
  printf '\nSee the Troubleshooting section of README.md, then run ./install.sh again.\n' >&2
  exit 1
}
# Runs a command with its output in the log; shows a spinner on a terminal.
run() {
  local label=$1; shift
  echo "\$ $*" >>"$LOG"
  if [[ -t 1 ]]; then
    "$@" >>"$LOG" 2>&1 &
    local pid=$! i=0 frames='|/-\'
    while kill -0 "$pid" 2>/dev/null; do
      printf '\r  %s %s ' "${frames:i++%4:1}" "$label"; sleep 0.2
    done
    printf '\r\033[K'
    wait "$pid"
  else
    info "$label"
    "$@" >>"$LOG" 2>&1
  fi
}
confirm() {
  [[ $ASSUME_YES -eq 1 || ! -t 0 ]] && return 0
  local reply
  read -r -p "  $1 [Y/n] " reply
  [[ -z "$reply" || "$reply" =~ ^[Yy] ]]
}
have() { command -v "$1" >/dev/null 2>&1; }
# version_ge A B: A >= B for dotted numbers (pure bash: macOS ships bash 3.2)
version_ge() {
  local IFS=. i a b
  read -r -a a <<<"$1"; read -r -a b <<<"$2"
  for i in 0 1 2; do
    local x=${a[i]:-0} y=${b[i]:-0}
    x=${x%%[!0-9]*}; y=${y%%[!0-9]*}
    (( ${x:-0} > ${y:-0} )) && return 0
    (( ${x:-0} < ${y:-0} )) && return 1
  done
  return 0
}

mkdir -p "$ROOT/build"
: >"$LOG"
printf '%sSifted installer%s  %s(log: build/install.log)%s\n' "$B" "$RST" "$DIM" "$RST"

# ----------------------------------------------------------------------------- 1. system
step "Checking your system"
OS="$(uname -s)"
IS_WSL=0
case "$OS" in
  Linux)
    grep -qi microsoft /proc/version 2>/dev/null && IS_WSL=1
    DISTRO="Linux"
    if [[ -r /etc/os-release ]]; then DISTRO="$(. /etc/os-release && echo "${PRETTY_NAME:-${NAME:-Linux}}")"; fi
    ok "$DISTRO$([[ $IS_WSL -eq 1 ]] && echo ' (Windows Subsystem for Linux)') on $(uname -m)" ;;
  Darwin)
    ok "macOS $(sw_vers -productVersion 2>/dev/null || echo '') on $(uname -m)" ;;
  MINGW*|MSYS*|CYGWIN*)
    die "This is a Windows shell (Git Bash, MSYS or Cygwin)." \
        "Sifted runs on Windows through WSL. Double-click install.cmd instead, or run it from PowerShell." ;;
  *)
    die "Unsupported system: $OS" "Sifted supports Linux, macOS and Windows (through WSL)." ;;
esac
SUDO=""
if [[ "$OS" == Linux && $(id -u) -ne 0 ]]; then SUDO="sudo"; fi

PM="" BREW_PREFIX=""
if [[ "$OS" == Darwin ]]; then
  BREW=""
  for b in brew /opt/homebrew/bin/brew /usr/local/bin/brew; do
    if have "$b" || [[ -x "$b" ]]; then BREW="$(command -v "$b" || echo "$b")"; break; fi
  done
  if [[ -n "$BREW" ]]; then
    eval "$("$BREW" shellenv)"
    PM=brew
    BREW_PREFIX="$(brew --prefix)"
  fi
  if ! xcode-select -p >/dev/null 2>&1; then
    warn "Apple's command line developer tools are missing."
    xcode-select --install >/dev/null 2>&1 || true
    die "Install the command line developer tools first." \
        "A window should have opened asking to install them. When that finishes, run ./install.sh again."
  fi
else
  for m in apt-get dnf yum pacman zypper; do
    if have "$m"; then PM="$m"; break; fi
  done
fi
[[ -n "$PM" ]] && ok "Package manager: $PM" || info "No supported package manager found"

# ----------------------------------------------------------------------------- 2. prerequisites
step "Checking prerequisites"
MISSING=()        # logical names: cmake ninja compiler python eigen gtest git
PY=""
for p in python3 python3.13 python3.12 python3.11 python3.10; do
  if have "$p" && "$p" -c 'import sys; sys.exit(sys.version_info < (3, 10))' 2>/dev/null; then PY="$(command -v "$p")"; break; fi
done
if [[ -n "$PY" ]]; then ok "Python $("$PY" -c 'import platform; print(platform.python_version())')"; else warn "Python 3.10+ not found"; MISSING+=(python); fi

CMAKE=""
if have cmake && version_ge "$(cmake --version | head -n1 | awk '{print $3}')" 3.21; then
  CMAKE="$(command -v cmake)"; ok "CMake $(cmake --version | head -n1 | awk '{print $3}')"
elif have cmake; then warn "CMake $(cmake --version | head -n1 | awk '{print $3}') is too old (3.21+ needed)"; MISSING+=(cmake)
else warn "CMake not found"; MISSING+=(cmake); fi

NINJA=""
for n in ninja ninja-build; do if have "$n"; then NINJA="$(command -v "$n")"; break; fi; done
if [[ -n "$NINJA" ]]; then ok "Ninja $("$NINJA" --version)"; else warn "Ninja not found"; MISSING+=(ninja); fi

# A compiler is usable if it builds C++20 with floating-point std::from_chars
# (GCC 11+, Clang/libc++ 20+, MSVC is not supported: the code uses POSIX sockets).
PROBE_DIR="$ROOT/build/probe"
mkdir -p "$PROBE_DIR"
cat >"$PROBE_DIR/probe.cpp" <<'EOF'
#include <charconv>
#include <span>
#include <string_view>
int main() {
  std::string_view s = "1.5";
  double d = 0;
  auto r = std::from_chars(s.data(), s.data() + s.size(), d);
  int a[2] = {0, 0};
  std::span<int> sp(a);
  return (r.ec == std::errc{} && d == 1.5 && sp.size() == 2) ? 0 : 1;
}
EOF
probe_cxx() {
  local cxx=$1
  have "$cxx" || [[ -x "$cxx" ]] || return 1
  "$cxx" -std=c++20 -o "$PROBE_DIR/probe" "$PROBE_DIR/probe.cpp" >>"$LOG" 2>&1 && "$PROBE_DIR/probe"
}
find_cxx() {
  CXX_FOUND=""
  local candidates=()
  [[ -n "${CXX:-}" ]] && candidates+=("$CXX")
  candidates+=(c++ g++ clang++)
  for v in 15 14 13 12 11; do candidates+=("g++-$v"); done
  if [[ -n "$BREW_PREFIX" ]]; then
    for v in 15 14 13 12 11; do candidates+=("$BREW_PREFIX/bin/g++-$v"); done
    candidates+=("$BREW_PREFIX/opt/llvm/bin/clang++")
  fi
  for c in "${candidates[@]}"; do
    if probe_cxx "$c"; then CXX_FOUND="$(command -v "$c" || echo "$c")"; return 0; fi
  done
  return 1
}
if find_cxx; then ok "C++ compiler: $("$CXX_FOUND" --version | head -n1)"
else warn "No C++20 compiler with full <charconv> support found"; MISSING+=(compiler); fi

has_header() {
  local h=$1 d
  for d in /usr/include /usr/local/include ${BREW_PREFIX:+"$BREW_PREFIX/include"}; do
    [[ -f "$d/$h" ]] && return 0
  done
  return 1
}
if has_header eigen3/Eigen/Core || { have pkg-config && pkg-config --exists eigen3; }; then ok "Eigen 3"
else info "Eigen 3 not installed"; MISSING+=(eigen); fi
if [[ $WITH_TESTS -eq 1 ]]; then
  if has_header gtest/gtest.h; then ok "GoogleTest"; else info "GoogleTest not installed"; MISSING+=(gtest); fi
fi
have git || MISSING+=(git)

# ----------------------------------------------------------------------------- 3. install
step "Installing what's missing"
pkgs_for() {  # package names for the logical names in $@, for $PM
  local out=() n
  for n in "$@"; do
    case "$PM:$n" in
      apt-get:compiler) out+=(build-essential g++) ;;   apt-get:cmake) out+=(cmake) ;;
      apt-get:ninja) out+=(ninja-build) ;;               apt-get:python) out+=(python3) ;;
      apt-get:eigen) out+=(libeigen3-dev) ;;             apt-get:gtest) out+=(libgtest-dev) ;;
      apt-get:git) out+=(git) ;;
      dnf:compiler|yum:compiler) out+=(gcc-c++ make) ;;  dnf:cmake|yum:cmake) out+=(cmake) ;;
      dnf:ninja|yum:ninja) out+=(ninja-build) ;;         dnf:python|yum:python) out+=(python3) ;;
      dnf:eigen|yum:eigen) out+=(eigen3-devel) ;;        dnf:gtest|yum:gtest) out+=(gtest-devel) ;;
      dnf:git|yum:git) out+=(git) ;;
      pacman:compiler) out+=(base-devel) ;;              pacman:cmake) out+=(cmake) ;;
      pacman:ninja) out+=(ninja) ;;                      pacman:python) out+=(python) ;;
      pacman:eigen) out+=(eigen) ;;                      pacman:gtest) out+=(gtest) ;;
      pacman:git) out+=(git) ;;
      zypper:compiler) out+=(gcc-c++) ;;                 zypper:cmake) out+=(cmake) ;;
      zypper:ninja) out+=(ninja) ;;                      zypper:python) out+=(python3) ;;
      zypper:eigen) out+=(eigen3-devel) ;;               zypper:gtest) out+=(gtest) ;;
      zypper:git) out+=(git) ;;
      brew:compiler) out+=(gcc) ;;                       brew:cmake) out+=(cmake) ;;
      brew:ninja) out+=(ninja) ;;                        brew:python) out+=(python@3.12) ;;
      brew:eigen) out+=(eigen) ;;                        brew:gtest) out+=(googletest) ;;
      brew:git) out+=(git) ;;
    esac
  done
  [[ ${#out[@]} -gt 0 ]] && printf '%s\n' "${out[@]}"
  return 0
}
pm_install() {
  case "$PM" in
    apt-get) run "Updating package lists" $SUDO apt-get update && \
             run "Installing $*" $SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y "$@" ;;
    dnf|yum) run "Installing $*" $SUDO "$PM" install -y "$@" ;;
    pacman)  run "Installing $*" $SUDO pacman -S --needed --noconfirm "$@" ;;
    zypper)  run "Installing $*" $SUDO zypper --non-interactive install "$@" ;;
    brew)    run "Installing $* (Homebrew)" brew install "$@" ;;
  esac
}

FETCH_DEPS=0
if [[ ${#MISSING[@]} -eq 0 ]]; then
  ok "Nothing to install"
else
  if [[ $SYSTEM_PKGS -eq 1 && -n "$PM" ]]; then
    PKGS=()
    while IFS= read -r line; do [[ -n "$line" ]] && PKGS+=("$line"); done < <(pkgs_for "${MISSING[@]}")
    info "Needed: ${PKGS[*]}"
    [[ -n "$SUDO" ]] && info "Your password may be requested (sudo) to install them."
    if confirm "Install them with $PM now?"; then
      if pm_install "${PKGS[@]}"; then
        ok "Installed ${PKGS[*]}"
      else
        warn "The package manager failed (details in build/install.log); trying to continue without it."
      fi
    else
      warn "Skipped. Will try to continue without them."
    fi
  elif [[ "$OS" == Darwin && -z "$PM" ]]; then
    die "Homebrew is needed to install Sifted's build tools on macOS." \
        "Install it from https://brew.sh (one command), then run ./install.sh again."
  fi

  # Re-check, then fall back where we can.
  if [[ ${#MISSING[@]} -gt 0 ]]; then
    have git && GIT_OK=1 || GIT_OK=0
    if [[ -z "$PY" ]]; then
      for p in python3 python3.13 python3.12 python3.11 python3.10; do
        if have "$p" && "$p" -c 'import sys; sys.exit(sys.version_info < (3, 10))' 2>/dev/null; then PY="$(command -v "$p")"; break; fi
      done
    fi
    [[ -n "$PY" ]] || die "Python 3.10 or newer is required." "Install it from https://www.python.org/downloads/ or your package manager."
    if [[ -z "$CMAKE" ]] && have cmake && version_ge "$(cmake --version | head -n1 | awk '{print $3}')" 3.21; then CMAKE="$(command -v cmake)"; fi
    if [[ -z "$NINJA" ]]; then for n in ninja ninja-build; do have "$n" && NINJA="$(command -v "$n")" && break; done; fi
    if [[ -z "$CMAKE" || -z "$NINJA" ]]; then
      # Self-contained fallback: CMake and Ninja from PyPI in a private venv.
      TOOLS="$ROOT/build/tools"
      if run "Installing CMake and Ninja into build/tools" bash -c \
          "'$PY' -m venv '$TOOLS' && '$TOOLS/bin/pip' install --quiet --upgrade 'cmake>=3.21' ninja"; then
        [[ -z "$CMAKE" ]] && CMAKE="$TOOLS/bin/cmake"
        [[ -z "$NINJA" ]] && NINJA="$TOOLS/bin/ninja"
        ok "CMake and Ninja installed privately (build/tools)"
      else
        die "CMake 3.21+ and Ninja are required." "Install them with your package manager (e.g. 'sudo apt install cmake ninja-build') and run ./install.sh again."
      fi
    fi
    if [[ -z "${CXX_FOUND:-}" ]] && ! find_cxx; then
      if [[ "$OS" == Darwin ]]; then
        die "No suitable C++ compiler." "Apple's compiler is too old for this project's C++20 features. Run 'brew install gcc', then ./install.sh again."
      fi
      die "No suitable C++ compiler (GCC 11+ or Clang 20+ with C++20 <charconv>)." \
          "Install one (e.g. 'sudo apt install g++') and run ./install.sh again."
    fi
    if ! has_header eigen3/Eigen/Core && ! { have pkg-config && pkg-config --exists eigen3; }; then
      [[ $GIT_OK -eq 1 ]] || die "Eigen 3 is missing and git isn't available to download it." "Install Eigen ('libeigen3-dev' / 'eigen') or git, then run ./install.sh again."
      FETCH_DEPS=1
      info "Eigen will be downloaded during the build (needs internet)."
    fi
    if [[ $WITH_TESTS -eq 1 ]] && ! has_header gtest/gtest.h; then
      [[ $GIT_OK -eq 1 ]] || die "GoogleTest is missing and git isn't available to download it."
      FETCH_DEPS=1
      info "GoogleTest will be downloaded during the build (needs internet)."
    fi
  fi
fi
[[ -n "$CMAKE" ]] || CMAKE="$(command -v cmake)"
[[ -n "$NINJA" ]] || { for n in ninja ninja-build; do have "$n" && NINJA="$(command -v "$n")" && break; done; }
[[ -n "${CXX_FOUND:-}" ]] || find_cxx || die "No suitable C++ compiler found after installing."

# ----------------------------------------------------------------------------- 4. build
step "Building the fusion service"
CACHE="$BUILD_DIR/CMakeCache.txt"
if [[ $CLEAN -eq 1 && -d "$BUILD_DIR" ]]; then rm -rf "$BUILD_DIR"; info "Removed the previous build"; fi
if [[ -f "$CACHE" ]]; then
  # CMake can't switch compilers in an existing build folder: start that folder over.
  prev="$(sed -n 's/^CMAKE_CXX_COMPILER:[A-Z]*=//p' "$CACHE")"
  if [[ -n "$prev" && "$(realpath "$prev" 2>/dev/null || echo "$prev")" != "$(realpath "$CXX_FOUND" 2>/dev/null || echo "$CXX_FOUND")" ]]; then
    rm -rf "$BUILD_DIR"; info "Compiler changed: starting a fresh build"
  fi
fi
CONFIGURE=("$CMAKE" --preset "$PRESET" "-DCMAKE_CXX_COMPILER=$CXX_FOUND" "-DCMAKE_MAKE_PROGRAM=$NINJA"
           "-DSIFTED_BUILD_TESTS=$([[ $WITH_TESTS -eq 1 ]] && echo ON || echo OFF)"
           "-DSIFTED_FETCH_DEPS=$([[ $FETCH_DEPS -eq 1 ]] && echo ON || echo OFF)")
# shellcheck disable=SC2206  # word-splitting of the user's extra arguments is intended
[[ -n "${SIFTED_CMAKE_ARGS:-}" ]] && CONFIGURE+=(${SIFTED_CMAKE_ARGS})
run "Configuring" "${CONFIGURE[@]}" || die "Configuring the build failed."
ok "Configured (build/$PRESET)"
JOBS="$( (have nproc && nproc) || sysctl -n hw.ncpu 2>/dev/null || echo 2)"
run "Compiling with $JOBS jobs (a few minutes the first time)" "$CMAKE" --build --preset "$PRESET" -j "$JOBS" \
  || die "The build failed." "Scroll up in build/install.log for the first 'error:' line."
FUSIOND="$BUILD_DIR/src/apps/sifted_fusiond"
[[ -x "$FUSIOND" ]] || die "The build finished but $FUSIOND is missing."
ok "Built $("$FUSIOND" --version)"

# ----------------------------------------------------------------------------- 5. verify
step "Checking the installation"
if [[ $WITH_TESTS -eq 1 ]]; then
  run "Running the C++ tests" ctest --test-dir "$BUILD_DIR" --output-on-failure -j "$JOBS" \
    || die "Some C++ tests failed." "See build/install.log."
  ok "C++ tests passed"
  py_tests() { cd "$ROOT/sim" && PYTHONPATH=. "$PY" -m unittest discover -s tests; }
  run "Running the Python tests" py_tests \
    || die "Some Python tests failed." "See build/install.log."
  ok "Python tests passed"
fi
# End-to-end smoke test: a short scenario through the real daemon, then replay it.
SMOKE="$ROOT/build/smoke"
rm -rf "$SMOKE"; mkdir -p "$SMOKE"
smoke() {
  local port
  port="$("$PY" -c 'import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1])')"
  (cd "$ROOT/sim" && "$PY" -m sifted_sim.generate crossing_pair "$SMOKE/scenario") || return 1
  "$FUSIOND" --sensors "$SMOKE/scenario/sensors.cfg" --listen "127.0.0.1:$port" --ws-port 0 \
    --record "$SMOKE/run.siftlog" --output "$SMOKE/live.txt" --stats-interval 0 &
  local pid=$!
  sleep 0.5
  (cd "$ROOT/sim" && "$PY" -m sifted_sim.send "$SMOKE/scenario" --target "127.0.0.1:$port" --speed 0) || { kill "$pid"; return 1; }
  sleep 1
  kill -INT "$pid"; wait "$pid" || return 1
  "$FUSIOND" --replay "$SMOKE/run.siftlog" --output "$SMOKE/replay.txt" --ws-port 0 || return 1
  grep -q '"type":"sifted.tracks"' "$SMOKE/live.txt" && grep -q ' updated ' "$SMOKE/live.txt" \
    && cmp -s "$SMOKE/live.txt" "$SMOKE/replay.txt"
}
run "Running a short scenario end to end" smoke || die "The end-to-end check failed." "The daemon, simulator or replay did not behave as expected; see build/install.log."
ok "End-to-end check passed (live run and replay identical)"
rm -rf "$SMOKE" "$PROBE_DIR"
"$PY" "$ROOT/app/launcher.py" --check >>"$LOG" 2>&1 && ok "Launcher check passed" || warn "Launcher check reported problems (see build/install.log)"
chmod +x "$ROOT/sifted" "$ROOT/install.sh" 2>/dev/null || true

# ----------------------------------------------------------------------------- 6. shortcut
step "Adding a shortcut"
if [[ $SHORTCUT -eq 0 ]]; then
  info "Skipped"
elif [[ "$OS" == Darwin ]]; then
  DESK="$HOME/Desktop/Sifted.command"
  cat >"$DESK" <<EOF
#!/bin/bash
# Starts Sifted (created by install.sh; safe to delete).
cd "$ROOT" && exec ./sifted
EOF
  chmod +x "$DESK"
  ok "Desktop shortcut: Sifted.command (double-click to start)"
elif [[ $IS_WSL -eq 1 ]]; then
  info "On Windows, install.cmd creates the desktop shortcut."
else
  APPS="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
  mkdir -p "$APPS"
  cat >"$APPS/sifted.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=Sifted
Comment=Multi-sensor track fusion
Exec="$ROOT/sifted" --background
Path=$ROOT
Icon=$ROOT/app/assets/sifted.png
Terminal=false
Categories=Science;Education;
EOF
  chmod +x "$APPS/sifted.desktop"
  have update-desktop-database && update-desktop-database "$APPS" >/dev/null 2>&1 || true
  ok "Added Sifted to your applications menu"
fi

printf '\n%s%s✓ Sifted is installed.%s\n\n' "$B" "$GRN" "$RST"
if [[ $FROM_WINDOWS -eq 1 ]]; then
  echo "  Start it from the Sifted shortcut on your desktop."
else
  echo "  Start it with:   ./sifted"
  [[ "$OS" == Darwin && $SHORTCUT -eq 1 ]] && echo "  or double-click:  Sifted.command on your Desktop"
  [[ "$OS" == Linux && $IS_WSL -eq 0 && $SHORTCUT -eq 1 ]] && echo "  or open:          Sifted, from your applications menu"
fi
echo "  It opens in your browser at http://127.0.0.1:8000/"
echo

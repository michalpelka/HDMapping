# Contributing to HDMapping

Thank you for your interest in contributing to HDMapping! This repository contains C++/CMake core libraries and multiple apps, with Python bindings via PyBind and helper scripts. To keep contributions smooth and maintainable, please follow the guidelines below.

## Getting Started
- **Fork and clone:** Fork the repo, then clone your fork.
- **Submodules/3rdparty:** The repo includes `3rdparty` sources and CMake modules. No action is usually required, but ensure submodules are initialized if used in your branch.
- **Build requirements:**
	- CMake ≥ 4.0
	- A C++20 compiler (GCC ≥ 9 or Visual Studio 2017)
	- `clang-format` for C++ formatting

## Branching & Workflow
- **Create a feature branch:**
	- Naming: `feature/<short-name>`, `fix/<short-name>`, `docs/<short-name>`, `chore/<short-name>`.
- **Keep changes focused:** Small, cohesive PRs are reviewed faster.
- **Rebase before PR:** Rebase onto `main` to keep history clean.

## Commit Messages
- Follow **Conventional Commits**:
	- `feat: add TLS registration option`
	- `fix(core): guard null pointer in loader`
	- `docs(pybind): clarify example usage`
	- `chore(cmake): bump minimum required version`
- Use the **imperative mood**, include **scope** when relevant, and **link issues** (e.g., `Fixes #123`).

## Pull Requests
- **Checklist:**
	- Code builds locally on Linux.
	- New code is formatted and linted.
	- Tests added/updated where applicable.
	- Documentation updated (README, inline docs, or `docs/`).
	- CI passes (for all platforms)
    - GUI changes needs screenshots and review on antoher platform.
- **Description:**
	- Explain the problem and solution clearly.
	- Note any performance or API impacts.
	- Include screenshots for GUI changes.

## Code Style & Standards
- **C++**
	- Standard: C++17.
	- Formatting: run `clang-format`.
	- Headers: prefer `#pragma once` for include guards.
	- Naming: `PascalCase` for types, `camelCase` for functions/variables, `ALL_CAPS` for macros.
	- Error handling: use `std::optional`, `expected`-like patterns or clear return codes; avoid silent failures.
	- Dependencies: prefer existing libraries under `3rdparty/`; avoid introducing large new dependencies without discussion.
- **CMake**
	- Minimum version at top-level `CMakeLists.txt` only; avoid duplicating across subdirs.
	- Use `target_link_libraries`, `target_include_directories`, and `target_compile_features` on targets; avoid directory-wide settings.
	- Keep options under clear `HDMAPPING_*` names; provide sensible defaults.
	- Structure new apps under `apps/<app-name>/` with a local `CMakeLists.txt` using a simple target.


## Building Locally

Refer to README.md

## Formatting

On Ubuntu:
```bash
cd HDMapping
sudo apt-get update
sudo apt-get install -y clang-format
clang-format --version

find . \
  -path ./3rdparty -prune -o \
  -path ./3rdpartyBinary -prune -o \
  -regex '.*\.\(h\|hh\|hpp\|hxx\|c\|cc\|cpp\|cxx\)' -print0 \
  | xargs -0 clang-format -i
```

On Windows:
```powershell
# Install via winget
winget install LLVM.LLVM
clang-format --version

# Format all C/C++ files in-place
Get-ChildItem -Recurse -File -Include *.h,*.hh,*.hpp,*.hxx,*.c,*.cc,*.cpp,*.cxx `
  | Where-Object { $_.FullName -notmatch '\\3rdparty\\' -and $_.FullName -notmatch '\\3rdpartyBinary\\' } `
  | ForEach-Object { clang-format -i $_.FullName }

```

## Testing
- Add tests adjacent to the component when possible. C++ tests belong under `core/tests` or similar (create if missing) and use a lightweight framework (e.g., GoogleTest) if already present; otherwise add minimal self-checks.
- For Python, prefer `pytest` with tests in `pybind/tests` or package-specific folders.
- Ensure `test_version.cpp` and any existing smoke tests still pass.
- Provide data-free tests or tiny fixtures; large sample data should not be committed unless essential.

## Performance & Data
- Be mindful of memory usage (see `doc/virtual_memory.md`).
- For large data processing, document memory expectations and chunking strategies.
- Do not commit large binaries or datasets; use links or document how to obtain them.
- Use submodules instead of commiting 3rd party code. Avoid binaries, use CMake logic.

## Security & Licensing
- Follow the project **LICENSE**. Do not add third-party code without compatible licenses.

## Issue Reporting & Feature Requests
- **Bugs:** include OS, compiler, CMake version, steps to reproduce, logs, and minimal data or synthetic examples.
- **Features:** describe the use case, alternatives considered, and potential impacts.
- Use labels when opening issues if available.

## Code of Conduct
We follow the Contributor Covenant Code of Conduct. Be respectful and collaborative.

## Developer Certificate of Origin (DCO)
By contributing, you agree to the DCO. Optionally use `Signed-off-by: Name <email>` in commit messages if requested by maintainers.

## Maintainer Tips (for reviewers)
- Ensure PRs stay scoped; request splits if too broad.
- Confirm build and basic runtime on Linux.
- Check formatting and CMake target hygiene.
- Ask for tests or examples when behavior changes.

## Questions
If anything is unclear, open a discussion or issue. We appreciate your contributions!

#!/usr/bin/env python3
"""Run clang-tidy on the project's own sources (changed files by default, or all).

Uses the root compile_commands.json (last configured preset). Output is in
`file:line:col: severity: message [check]` form for editor problem matchers.

  tools/clang_tidy.py            # changed + untracked files vs HEAD
  tools/clang_tidy.py --all      # every project source in the compile DB
  tools/clang_tidy.py FILE...    # specific files
"""
import argparse
import glob
import json
import os
import shlex
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SOURCE_DIRS = ("main", "components")
SOURCE_EXTS = (".c", ".cc", ".cpp")
HEADER_EXTS = (".h", ".hpp")
# GCC-only flags clang does not understand (keep in sync with .clangd).
REMOVED_FLAGS = {
    "-fno-shrink-wrap",
    "-fstrict-volatile-bitfields",
    "-fno-tree-switch-conversion",
    "-fzero-init-padding-bits=all",
    "-fno-malloc-dce",
}


def find_clang_tidy():
    if env := os.environ.get("CLANG_TIDY"):
        return env
    found = sorted(glob.glob(os.path.expanduser("~/.espressif/tools/esp-clang/*/esp-clang/bin/clang-tidy")))
    if found:
        return found[-1]
    if path := shutil.which("clang-tidy"):
        return path
    sys.exit("clang-tidy not found (set CLANG_TIDY)")


def keep_arg(arg):
    return arg not in REMOVED_FLAGS and not arg.startswith("-Werror")


_driver_cache = {}


def driver_system_args(args, directory):
    """Like clangd's --query-driver: ask GCC for its target and system include dirs.

    clang ignores GCC -specs (e.g. picolibc.specs), so without this it picks the wrong libc headers.
    """
    compiler = args[0]
    lang = "c++" if compiler.endswith("++") else "c"
    specs = tuple(a for a in args if a.startswith("-specs="))
    key = (compiler, lang, specs)
    if key not in _driver_cache:
        result = subprocess.run([compiler, *specs, "-E", "-x", lang, "-v", os.devnull],
                                cwd=directory, capture_output=True, text=True)
        lines = result.stderr.splitlines()
        dirs, inside = [], False
        for line in lines:
            if line.startswith("#include <...> search starts here:"):
                inside = True
            elif line.startswith("End of search list."):
                break
            elif inside:
                dirs.append(os.path.normpath(line.strip()))
        target = next((l.split(":", 1)[1].strip() for l in lines if l.startswith("Target:")), None)
        extra = ["-nostdlibinc"] + [f"-isystem{d}" for d in dirs]
        _driver_cache[key] = extra + ([f"--target={target}"] if target else [])
    return _driver_cache[key]


def expand_response_files(args, directory):
    """Inline `@file` args (ESP-IDF keeps toolchain flags like -specs/-march there)."""
    expanded = []
    for arg in args:
        if arg.startswith("@"):
            path = Path(directory, arg[1:].strip('"'))
            expanded += expand_response_files(shlex.split(path.read_text()), directory)
        else:
            expanded.append(arg)
    return expanded


def filtered_db(out_dir):
    db_path = ROOT / "compile_commands.json"
    if not db_path.exists():
        sys.exit("compile_commands.json not found: build a preset first (idf.py --preset default build)")
    entries = json.loads(db_path.read_text())
    for entry in entries:
        args = entry["arguments"] if "arguments" in entry else shlex.split(entry["command"])
        args = [a for a in expand_response_files(args, entry["directory"]) if keep_arg(a)]
        path = Path(entry["directory"], entry["file"]).resolve()
        if is_project_file(path):  # only project files get checked; skip querying for the rest
            args = args[:1] + driver_system_args(args, entry["directory"]) + args[1:]
        entry.pop("command", None)
        entry["arguments"] = args
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "compile_commands.json").write_text(json.dumps(entries))
    return {str(Path(e["directory"], e["file"]).resolve()) for e in entries}


def is_project_file(path):
    rel = path.relative_to(ROOT) if path.is_relative_to(ROOT) else None
    return rel is not None and rel.parts and rel.parts[0] in SOURCE_DIRS


def changed_files():
    def git(*args):
        return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, check=True).stdout.split("\n")

    names = git("diff", "--name-only", "HEAD") + git("ls-files", "--others", "--exclude-standard")
    return [ROOT / n for n in names if n]


def to_sources(paths, db_files):
    sources = set()
    for path in paths:
        path = path.resolve()
        if path.suffix in HEADER_EXTS:
            # A header has no compile entry: check the same-named source next to it, if any.
            candidates = [path.with_suffix(ext) for ext in SOURCE_EXTS]
            candidates += [path.parent.parent / (path.stem + ext) for ext in SOURCE_EXTS]  # include/x.h -> x.c
            path = next((c for c in candidates if str(c) in db_files), path)
        if str(path) in db_files and is_project_file(path):
            sources.add(str(path))
    return sorted(sources)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="*", help="files to check (default: changed files)")
    parser.add_argument("--all", action="store_true", help="check every project source")
    parser.add_argument("-j", "--jobs", type=int, default=os.cpu_count())
    args = parser.parse_args()

    clang_tidy = find_clang_tidy()
    db_dir = ROOT / "build" / "clang-tidy"
    db_files = filtered_db(db_dir)

    if args.all:
        sources = sorted(f for f in db_files if is_project_file(Path(f)))
    else:
        sources = to_sources([Path(f) for f in args.files] if args.files else changed_files(), db_files)
    if not sources:
        print("No project sources to check.")
        return 0

    # .clang-tidy's HeaderFilterRegex is relative and never matches absolute paths; scope to our dirs.
    header_filter = "^" + str(ROOT) + "/(" + "|".join(SOURCE_DIRS) + ")/"
    print(f"clang-tidy: checking {len(sources)} file(s) with {clang_tidy}", flush=True)

    # A C component's headers are checked via its C sources only; C++ includers would apply C++ rules.
    c_dirs = sorted({str(Path(f).parent) for f in db_files if f.endswith(".c") and is_project_file(Path(f))})
    c_header_filter = "^(" + "|".join(c_dirs) + ")/" if c_dirs else None
    # Vendored, unmodified code (components/*/third_party, often a git submodule that can't hold a
    # .clang-tidy) is never reported via its headers, and its sources get bug-finding checks only.
    vendor_header_filter = ".*/third_party/.*"

    def run(source):
        excludes = [vendor_header_filter]
        if c_header_filter and not source.endswith(".c"):
            excludes.append(c_header_filter)
        cmd = [clang_tidy, "-p", str(db_dir), "--quiet", f"--header-filter={header_filter}",
               f"--exclude-header-filter={'|'.join(excludes)}", source]
        if "/third_party/" in source:
            cmd.insert(-1, "--checks=-*,clang-analyzer-*")
        return subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)

    failed = False
    seen = set()  # header diagnostics repeat for every source that includes the header
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for source, result in zip(sources, pool.map(run, sources)):
            print(f"--- {os.path.relpath(source, ROOT)}", flush=True)
            block = []
            for line in result.stdout.splitlines():
                if ": warning: " in line or ": error: " in line:
                    if block and block[0] not in seen:
                        seen.add(block[0])
                        print("\n".join(block))
                    block = [line]
                elif block:
                    block.append(line)
            if block and block[0] not in seen:
                seen.add(block[0])
                print("\n".join(block))
            failed |= result.returncode != 0
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

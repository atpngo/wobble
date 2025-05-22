import os
import sys
import shutil
import subprocess
import argparse


def rebuild(root, build_dir, temp_dir):
    # delete old build/
    print("Deleting old build files...")
    if os.path.exists(build_dir):
        shutil.rmtree(build_dir)

    # make new build/ + temp/
    print("Creating new build dir...")
    os.makedirs(temp_dir, exist_ok=True)

    # create shared libs
    print("Building shared cpp libraries...")
    subprocess.check_call(
        [
            sys.executable,
            "setup.py",
            "build_ext",
            "--build-lib",
            build_dir,
            "--build-temp",
            temp_dir,
        ],
        cwd=root,
    )
    print("Build complete!")


def run_sim(root):
    print("Starting simulation…")
    try:
        subprocess.run(
            [sys.executable, "src/sim.py"],
            cwd=root,
            check=True,
        )
    except KeyboardInterrupt:
        print("Terminating…")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Build &/or run the wobble simulation")
    parser.add_argument(
        "--no-build",
        action="store_true",
        help="Skip the C++ rebuild step and only run sim",
    )
    parser.add_argument(
        "--build-only",
        action="store_true",
        help="Only rebuild the C++ and then exit (do not run sim)",
    )
    args = parser.parse_args()

    root = os.path.dirname(__file__)
    build_dir = os.path.join(root, "build")
    temp_dir = os.path.join(build_dir, "temp")

    if not args.no_build:
        rebuild(root, build_dir, temp_dir)
    else:
        print("Skipping build step (--no-build)")

    if not args.build_only:
        run_sim(root)
    else:
        print("Build only requested (--build-only). Exiting.")


if __name__ == "__main__":
    main()

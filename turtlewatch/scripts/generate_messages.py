"""
Generate Python code from ROS message definitions using genpy.
This script processes all message packages in common_msgs/ without requiring catkin.
Disclaimer: Fully LLM generated
"""

import os
import subprocess
import sys
from pathlib import Path

# Base paths
BASE_DIR = Path(__file__).parent
COMMON_MSGS_DIR = BASE_DIR / "common_msgs"
ROS_MESSAGES_DIR = BASE_DIR / "ros_messages"
GENPY_SCRIPT_MSG = BASE_DIR / "genpy" / "scripts" / "genmsg_py.py"
GENPY_SCRIPT_SRV = BASE_DIR / "genpy" / "scripts" / "gensrv_py.py"

# Package processing order (dependencies first)
PACKAGE_ORDER = [
    "std_msgs",  # Must be first - contains Header
    "actionlib_msgs",
    "geometry_msgs",
    "sensor_msgs",
    "diagnostic_msgs",
    "nav_msgs",
    "shape_msgs",
    "stereo_msgs",
    "trajectory_msgs",
    "visualization_msgs",
]


def check_std_msgs():
    """Check if std_msgs exists locally."""
    std_msgs_dir = COMMON_MSGS_DIR / "std_msgs" / "msg"
    if std_msgs_dir.exists() and (std_msgs_dir / "Header.msg").exists():
        return std_msgs_dir
    return None


def generate_package_messages(
    package_name, msg_dir, srv_dir, output_dir, include_paths
):
    """Generate Python code for a single package's messages and services."""
    print(f"\n{'=' * 60}")
    print(f"Processing package: {package_name}")
    print(f"{'=' * 60}")

    success = True

    # Build include path arguments
    include_args = []
    for pkg, path in include_paths.items():
        include_args.extend(["-I", f"{pkg}:{path}"])

    # Generate messages
    if msg_dir and msg_dir.exists():
        msg_files = list(msg_dir.glob("*.msg"))
        if msg_files:
            print(f"\nGenerating {len(msg_files)} message(s)...")
            msg_output = output_dir / "msg"
            msg_output.mkdir(parents=True, exist_ok=True)

            cmd = (
                [
                    sys.executable,
                    str(GENPY_SCRIPT_MSG),
                    "-p",
                    package_name,
                    "-o",
                    str(msg_output),
                ]
                + include_args
                + [str(f) for f in msg_files]
            )

            print(f"Command: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode != 0:
                print(f"ERROR generating messages:")
                print(result.stderr)
                success = False
            else:
                print(f"✓ Generated messages in {msg_output}")

                # Generate __init__.py for messages
                init_cmd = [
                    sys.executable,
                    str(GENPY_SCRIPT_MSG),
                    "--initpy",
                    "-o",
                    str(msg_output),
                ]
                subprocess.run(init_cmd, capture_output=True)
                print(f"✓ Generated __init__.py")

    # Generate services
    if srv_dir and srv_dir.exists():
        srv_files = list(srv_dir.glob("*.srv"))
        if srv_files:
            print(f"\nGenerating {len(srv_files)} service(s)...")
            srv_output = output_dir / "srv"
            srv_output.mkdir(parents=True, exist_ok=True)

            cmd = (
                [
                    sys.executable,
                    str(GENPY_SCRIPT_SRV),
                    "-p",
                    package_name,
                    "-o",
                    str(srv_output),
                ]
                + include_args
                + [str(f) for f in srv_files]
            )

            print(f"Command: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode != 0:
                print(f"ERROR generating services:")
                print(result.stderr)
                success = False
            else:
                print(f"✓ Generated services in {srv_output}")

                # Generate __init__.py for services
                init_cmd = [
                    sys.executable,
                    str(GENPY_SCRIPT_SRV),
                    "--initpy",
                    "-o",
                    str(srv_output),
                ]
                subprocess.run(init_cmd, capture_output=True)
                print(f"✓ Generated __init__.py")

    return success


def main():
    print("ROS Message Python Code Generator")
    print("==================================\n")

    # Check for std_msgs locally
    std_msgs_path = check_std_msgs()
    if std_msgs_path:
        print(f"✓ Found std_msgs at: {std_msgs_path}")
    else:
        print("⚠ Warning: std_msgs not found in common_msgs/")
        print("  Messages using Header will fail.")

    # Build include paths (accumulate as we process packages)
    include_paths = {}
    if std_msgs_path:
        include_paths["std_msgs"] = str(std_msgs_path)

    # Process each package
    for package_name in PACKAGE_ORDER:
        package_dir = COMMON_MSGS_DIR / package_name

        if not package_dir.exists():
            print(f"⚠ Skipping {package_name} (not found)")
            continue

        msg_dir = package_dir / "msg"
        srv_dir = package_dir / "srv"
        output_dir = ROS_MESSAGES_DIR / package_name

        # Add current package to include paths
        if msg_dir.exists():
            include_paths[package_name] = str(msg_dir)

        # Generate
        success = generate_package_messages(
            package_name,
            msg_dir if msg_dir.exists() else None,
            srv_dir if srv_dir.exists() else None,
            output_dir,
            include_paths,
        )

        if not success:
            print(f"\n⚠ Package {package_name} had errors, but continuing...")

    print("\n" + "=" * 60)
    print("Generation complete!")
    print("=" * 60)
    print("\nGenerated Python packages are in:")
    print(f"  {ROS_MESSAGES_DIR}/")
    for package_name in PACKAGE_ORDER:
        output_dir = ROS_MESSAGES_DIR / package_name
        if output_dir.exists():
            msg_count = (
                len(list((output_dir / "msg").glob("_*.py")))
                if (output_dir / "msg").exists()
                else 0
            )
            srv_count = (
                len(list((output_dir / "srv").glob("_*.py")))
                if (output_dir / "srv").exists()
                else 0
            )
            print(f"    {package_name}: {msg_count} messages, {srv_count} services")


if __name__ == "__main__":
    main()

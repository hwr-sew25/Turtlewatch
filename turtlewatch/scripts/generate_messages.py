"""
Generate Python code from ROS message definitions using genpy.
This script processes all message packages in common_msgs/ without requiring catkin.
Disclaimer: Fully LLM generated
"""

import re
import subprocess
import sys
from pathlib import Path

# Base paths
BASE_DIR = Path(__file__).parent
LIBS_DIR = BASE_DIR / ".." / "libs"
COMMON_MSGS_DIR = LIBS_DIR / "common_msgs"
ROS_COMM_MSGS_DIR = LIBS_DIR / "ros_comm_msgs"
ROS_COMM_CLIENTS_DIR = LIBS_DIR / "ros_comm" / "clients"
ROS_MESSAGES_DIR = LIBS_DIR / ".." / "ros_msgs"
GENPY_SCRIPT_MSG = LIBS_DIR / "genpy" / "scripts" / "genmsg_py.py"
GENPY_SCRIPT_SRV = LIBS_DIR / "genpy" / "scripts" / "gensrv_py.py"

# Package processing order (dependencies first)
PACKAGE_ORDER = [
    "std_msgs",  # Must be first - contains Header
    "roscpp",
    "actionlib_msgs",
    "geometry_msgs",
    "sensor_msgs",
    "diagnostic_msgs",
    "nav_msgs",
    "shape_msgs",
    "stereo_msgs",
    "trajectory_msgs",
    "visualization_msgs",
    "rosgraph_msgs",
]


def fix_imports_in_directory(directory, package_name):
    """Fix imports in generated Python files to use relative imports."""
    print(f"Fixing imports in {directory}...")

    for py_file in directory.glob("_*.py"):
        content = py_file.read_text()

        # Prefer module imports (`import geometry_msgs.msg`) so attributes like
        # `geometry_msgs.msg.Point` are available in generated classes.
        packages = {
            "std_msgs",
            "geometry_msgs",
            "sensor_msgs",
            "nav_msgs",
            "diagnostic_msgs",
            "roscpp",
            "actionlib_msgs",
            "shape_msgs",
            "stereo_msgs",
            "trajectory_msgs",
            "visualization_msgs",
            "rosgraph_msgs",
            package_name,
        }

        for pkg in packages:
            content = re.sub(
                rf"^from {re.escape(pkg)}\.msg import \*$",
                f"import {pkg}.msg",
                content,
                flags=re.MULTILINE,
            )

        py_file.write_text(content)

    print(f"✓ Fixed imports in {directory}")


def find_package_dir(package_name):
    """Find package directory in libs/common_msgs or libs/ directly."""
    # First check in common_msgs
    pkg_dir = COMMON_MSGS_DIR / package_name
    if pkg_dir.exists():
        return pkg_dir

    # Then check in ros_comm_msgs (rosgraph_msgs, std_srvs, etc.)
    pkg_dir = ROS_COMM_MSGS_DIR / package_name
    if pkg_dir.exists():
        return pkg_dir

    # Then check in ros_comm/clients (roscpp messages/services)
    pkg_dir = ROS_COMM_CLIENTS_DIR / package_name
    if pkg_dir.exists():
        return pkg_dir

    # Then check in libs directly (for std_msgs)
    pkg_dir = LIBS_DIR / package_name
    if pkg_dir.exists():
        return pkg_dir

    return None


def generate_package_messages(
    package_name, msg_dir, srv_dir, output_dir, include_paths
):
    """Generate Python code for a single package's messages and services."""
    print("\n{'=' * 60}")
    print(f"Processing package: {package_name}")
    print("{'=' * 60}")

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
                print("ERROR generating messages:")
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
                print("✓ Generated __init__.py")

                # Fix imports in generated files to use relative imports
                fix_imports_in_directory(msg_output, package_name)

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
                print("ERROR generating services:")
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
                print("✓ Generated __init__.py")

                # Fix imports in generated files to use relative imports
                fix_imports_in_directory(srv_output, package_name)

    # Create a root __init__.py so the package can be imported directly
    # (e.g. `import roscpp.srv`) and to avoid collisions with any system
    # ROS installations on sys.path.
    if output_dir.exists():
        output_dir.mkdir(parents=True, exist_ok=True)
        init_lines = [f"# Auto-generated init for {package_name}"]
        if msg_dir and (output_dir / "msg").exists():
            init_lines.append("from . import msg")
            init_lines.append("from .msg import *")
        if srv_dir and (output_dir / "srv").exists():
            init_lines.append("from . import srv")
            init_lines.append("from .srv import *")
        init_lines.append("")
        (output_dir / "__init__.py").write_text("\n".join(init_lines))

    return success


def main():
    print("ROS Message Python Code Generator")
    print("==================================\n")

    # Build include paths (accumulate as we process packages)
    include_paths = {}

    # Process each package
    for package_name in PACKAGE_ORDER:
        package_dir = find_package_dir(package_name)

        if not package_dir:
            print(f"⚠ Skipping {package_name} (not found)")
            continue

        print(f"Found {package_name} at: {package_dir}")

        msg_dir = package_dir / "msg"
        srv_dir = package_dir / "srv"
        output_dir = ROS_MESSAGES_DIR / package_name

        # Add current package to include paths BEFORE generating
        # (so dependencies can find it)
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

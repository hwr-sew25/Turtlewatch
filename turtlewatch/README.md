# ROS Message Python Generation

This repository contains ROS message definitions and generated Python code using genpy (without catkin).

## Generated Messages

All Python message code has been successfully generated in the `ros_messages/` directory.

### Package Structure

All packages are located in `ros_messages/`:

- `std_msgs` - 3 messages: Standard message types (Header, ColorRGBA)
- `actionlib_msgs` - 4 messages: Action library messages
- `geometry_msgs` - 30 messages: Geometric primitives (Point, Pose, Transform, etc.)
- `sensor_msgs` - 28 messages + 2 services: Sensor data (Image, CameraInfo, LaserScan, etc.)
- `diagnostic_msgs` - 4 messages + 3 services: Diagnostic messages
- `nav_msgs` - 6 messages + 5 services: Navigation (Odometry, Path, OccupancyGrid, etc.)
- `shape_msgs` - 5 messages: Shape primitives (Mesh, Plane, etc.)
- `stereo_msgs` - 2 messages: Stereo camera messages
- `trajectory_msgs` - 5 messages: Trajectory messages
- `visualization_msgs` - 11 messages: Visualization markers

## Usage

### Importing Messages in Bridge Package

The `bridge` package is pre-configured to use the generated messages. Simply import the bridge package first:

```python
import bridge  # This automatically adds ros_messages to sys.path

# Then import messages normally
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
```

### Importing Messages in Other Scripts

For scripts outside the bridge package:

```python
# Add ros_messages directory to your Python path
import sys
sys.path.insert(0, 'ros_messages')

# Import messages
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import Image
from std_msgs.msg import Header
```

### Example: Creating a Point

```python
from geometry_msgs.msg import Point

point = Point()
point.x = 1.0
point.y = 2.0
point.z = 3.0
```

### Example: Creating an Image Message

```python
from sensor_msgs.msg import Image
from std_msgs.msg import Header

img = Image()
img.header = Header()
img.header.frame_id = "camera"
img.width = 640
img.height = 480
img.encoding = "rgb8"
```

## Regenerating Messages

To regenerate all Python message code:

```bash
python generate_messages.py
```

The script will:
1. Process messages in dependency order (std_msgs first)
2. Generate Python code for all .msg and .srv files
3. Create __init__.py files for each package
4. Output generated files to `ros_messages/PACKAGE/msg/` and `.../srv/`

## Manual Generation

You can also generate messages manually using genpy:

```bash
# For messages
python genpy/scripts/genmsg_py.py \
  -p PACKAGE_NAME \
  -o OUTPUT_DIR \
  -I package1:path/to/package1/msg \
  -I package2:path/to/package2/msg \
  path/to/file.msg

# For services
python genpy/scripts/gensrv_py.py \
  -p PACKAGE_NAME \
  -o OUTPUT_DIR \
  -I package1:path/to/package1/msg \
  path/to/file.srv

# Generate __init__.py
python genpy/scripts/genmsg_py.py --initpy -o OUTPUT_DIR
```

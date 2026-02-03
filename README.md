# Turtlewatch

Turtlewatch is a monitoring solution for TurtleBot3 robots, utilizing a ROS-Python bridge to capture and store telemetry data in an InfluxDB database. This data can then be visualized using Grafana.

## Getting Started

To get started, you need to clone the repository with its submodules.

**For a new copy:**

```bash
git clone --recurse-submodules -j8 git@github.com:hwr-sew25/Turtlewatch.git
```

**For an existing copy:**

```bash
git submodule update --init --recursive -j8
```

## Requirements

Before you begin, ensure you have the following installed:

1.  **uv**: Follow the installation instructions at [https://docs.astral.sh/uv/getting-started/installation/](https://docs.astral.sh/uv/getting-started/installation/).
2.  **Docker & Docker Compose**: Install via Docker Engine or Docker Desktop. See [https://docs.docker.com/get-started/get-docker/](https://docs.docker.com/get-started/get-docker/).
3.  **ROS**: The necessary ROS components for TurtleBot3.

## Infrastructure Setup

1.  **Configure Environment:**
    Copy the `.env.example` file to a new file named `.env`. This file will store your configuration variables.

2.  **Start InfluxDB:**
    ```bash
    docker compose up -d influxdb3-core
    ```

3.  **Create InfluxDB Token:**
    Generate an admin token for InfluxDB:
    ```bash
    docker compose exec influxdb3-core influxdb3 create token --admin
    ```
    Copy the generated token and paste it into your `.env` file as the value for `INFLUXDB_TOKEN`.

4.  **Initialize the Database:**
    Make the initialization script executable and run it:
    ```bash
    chmod +x scripts/init_db.sh
    ./scripts/init_db.sh
    ```

5.  **Start All Services:**
    Launch the rest of the Docker containers:
    ```bash
    docker compose up -d
    ```

## Verification

Once all services are running, you can verify the setup:

-   **Grafana:** Access Grafana at `http://localhost:3000`. Log in with `admin` / `admin`. The InfluxDB datasource and a dashboard should be pre-configured.
-   **InfluxDB Explorer:** The InfluxDB Explorer is available at `http://localhost:8888` and the database should also be pre-configured.
-   **Python Bridge (Mock Mode):** If `MOCK=TRUE` is set in your `.env` file, you can run the Python bridge in mock mode:
    ```bash
    cd turtlewatch
    uv run python -m bridge.main
    ```
    You should then see data appearing on the Grafana dashboard.

## Plugin Architecture

The Python bridge uses a lightweight plugin system to map ROS topics to InfluxDB measurements.

-   **Locations:** Plugins live in `turtlewatch/bridge/plugins/default_plugins` and `turtlewatch/bridge/plugins/custom_plugins`. The default plugins only use standard ROS message types, while custom plugins are for data or topics defined by other teams that are not ROS defaults.
-   **Discovery:** `bridge.plugin_loader.load_plugins()` imports every module in those folders, instantiates each `Plugin` subclass, and keeps only `is_enabled = True` instances.
-   **Execution:** `bridge.main` wires each active plugin into a `ThrottledSubscriber`, which either subscribes to the ROS topic or runs the plugin's `mock_generator` when `MOCK=TRUE`.
-   **Defaults:** `Plugin.callback()` converts the ROS message into an InfluxDB point via `ros_msg_to_influx_point`, using `topic_name` as the measurement name and `tags` for metadata.
-   **Examples:** Copy the example plugins in `turtlewatch/bridge/plugins/default_plugins/example.py` and `turtlewatch/bridge/plugins/custom_plugins/example.py` to get started.

To add a new plugin, create a new module in one of the plugin folders, subclass `Plugin`, define `topic_name`, `ros_msg_type`, `interval`, `tags`, and implement `mock_generator` (override `callback` only if you need custom processing).

## Running the Example

Now you can run the example to see Turtlewatch in action.

1.  **Start ROS Core:**
    ```bash
    roscore
    ```

2.  **Start Gazebo with TurtleBot3 World:**
    **Important:** You must use the `turtlebot3_world` map so that the Grafana Live-Map matches the simulation.

    ```bash
    export TURTLEBOT3_MODEL=waffle
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
    ```

3.  **Start the Python Bridge:**
    This assumes you are in the `turtlewatch/` directory and the database name is `dev`.
    ```bash
    cd turtlewatch
    uv run python -m bridge.main
    ```

4.  **Query Data in InfluxDB Explorer:**
    - Go to your InfluxDB Explorer at `http://localhost:8888`.
    - On the left side, click `Query Data` and then `Data Explorer`.
    - Run this simple query to see the data:
      ```sql
      select * from odometry
      ```

## Generating Message Files

To generate the ROS message files, run the following command. The output will be in the `libs/ros_msgs` directory.

```bash
uv run python scripts/generate_messages.py
```

# Turtlewatch 

## Requirements
1. Install uv (https://docs.astral.sh/uv/getting-started/installation/)
2. Docker + docker compose (e.g. through Docker Desktop: https://docs.docker.com/get-started/get-docker/)
3. The ROS stuff

## Setup Infra
### 1. Create Secret files 

``` bash
touch db_session_key.txt influxdb_token.txt
```

### 2. Start influxdb

``` bash
docker compose up -d influxdb3-core
```

### 3. Create influxdb token

``` bash
docker compose exec influxdb3-core influxdb3 create token --admin
```

Copy the token and paste it into `influxdb_token.txt`

### 4. db session key
Take some string and put it into the `db_session_key.txt` \
Example String: 
```
wSsAfVOmte9H015v4xGC74KmnQ1bUMO9
```

### 5. Start the rest.
Now Start 
``` bash
docker compose up -d
```

### 6. Setup Explorer
1. Go to `http://localhost:8888`. 
2. Click on **CONFIGURE SERVER** -> then on **Connect you first Server**\
Server Name: can be anything e.g. `dev` \
Server Url: `influxdb3-core:8181` \
Token: Copy and paste the token from your `influxdb_token.txt` \

3. In the left menu click **Manage Databases** -> then click **Create New**
4. Name it something e.g. `dev` and create it.

### 6. Setup Login to Grafan
Go to `http://localhost:300` and login with: \
Username: `admin` \
Password: `admin` \
You can change the password or just set it to admin again.

#### Add InfluxDB datasource
1. In the middle of the screen, click on **Add your first Datasource**.
2. Select or search for **influxdb**
3. Query language: `SQL`
4. HTTP->URL: `http://influxdb3-core:8181`
5. InfluxDB Details->Database: {the database name you created in the explorer e.g. `dev`}
6. Insecure Connection: toggle to true
7. Click **Save and Test** it should say "OK", now you can start using it


## Run the example
**This is a 100% LLM generated a proof of concept and will be thrown away**
1. Start `roscore`
2. Start `gazebo` (e.g. `roslaunch turtlebot3_gazebo turtlebot3_world.launch`)
3. Start python project 
This assumes that you are in the `turtlewatch/` dir and the database name is `dev`.
``` bash
cd turtlewatch
uv run python src/main.py
```
4. Go to your influxdb explorer (`http://localhost:8888`)
5. On the left side click `Query Data` and then `Data Explorer`
6. Run this simple query  

``` sql
select * from odometry
```


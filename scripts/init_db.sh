#!/usr/bin/env bash
source .env
source ../.env

curl -i -X POST "$INFLUXDB_URL/api/v3/configure/database" \
  -H "Authorization: Bearer $INFLUXDB_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"db": "'"$INFLUXDB_DB_NAME"'"}'


curl -i -X POST "$INFLUXDB_URL/api/v3/configure/database" \
  -H "Authorization: Bearer $INFLUXDB_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"db": "'"$INFLUXDB_DB_SESSIONS_NAME"'",
       "retention_period": null}'

# --- Create InfluxDB Explorer Configuration File ---
echo "Creating InfluxDB Explorer config file..."

# Define the path for the config file
CONFIG_DIR="influxdb3_explorer/config"
CONFIG_FILE="$CONFIG_DIR/config.json"

# Create the config directory if it doesn't exist to prevent errors
mkdir -p "$CONFIG_DIR"

# Write the config.json file, substituting the token from the environment
cat <<EOF > "$CONFIG_FILE"
{
  "DEFAULT_INFLUX_SERVER": "http://influxdb3-core:8181",
  "DEFAULT_INFLUX_DATABASE": "dev",
  "DEFAULT_API_TOKEN": "${INFLUXDB_TOKEN}",
  "DEFAULT_SERVER_NAME": "$INFLUXDB_NAME"
}
EOF

echo "Successfully created $CONFIG_FILE"

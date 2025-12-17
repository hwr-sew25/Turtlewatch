#!/usr/bin/env bash
source .env
source ../.env

curl -i -X POST "$INFLUXDB_URL/api/v3/configure/database" \
  -H "Authorization: Bearer $INFLUXDB_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"db": "dev"}'

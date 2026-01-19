#!/bin/bash
set -e

export HOME="${HOME:-/tmp}"

# 既存のROSエントリーポイントに処理を渡す
exec /ros_entrypoint.sh "$@"

#!/bin/bash
set -e
exec jupyter lab --ip=0.0.0.0 --port="${JUPYTER_PORT:-8889}" \
  --ServerApp.port_retries=0 --no-browser --allow-root \
  --IdentityProvider.token="${JUPYTER_TOKEN:-surface}" \
  --ServerApp.root_dir=/opt/demo

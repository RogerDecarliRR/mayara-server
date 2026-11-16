#!/usr/bin/env bash
# Para o compose lab do mayara-server (--emulator).
# Host-only — NÃO copiar para a imagem.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
COMPOSE_FILE="$ROOT/docker/docker-compose.lab.yml"
CONTAINER="mayara-local"

echo "==> docker compose down …"
docker compose -f "$COMPOSE_FILE" down

echo "stack parada (container $CONTAINER removido se existia)."
echo "Parar sem apagar a imagem: ok — rebuild só com ./scripts/dev/up-local.sh --build"

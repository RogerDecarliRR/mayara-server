#!/usr/bin/env bash
# Sobe mayara-server local (compose lab + --emulator). Host-only.
# Default: reusa mayara-server:local; se faltar, puxa GHCR e marca como :local.
# --build = rebuild do Dockerfile do clone (Rust; pode demorar / falhar em apk lento).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
COMPOSE_FILE="$ROOT/docker/docker-compose.lab.yml"
CONTAINER="mayara-local"
IMAGE_LOCAL="mayara-server:local"
IMAGE_GHCR="ghcr.io/marineyachtradar/mayara-server:latest"
DEFAULT_PORT="6502"
HEALTH_TIMEOUT_SEC=90
FORCE_BUILD=0

die() {
  echo "erro: $*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Uso: ./scripts/dev/up-local.sh [--build] [-h|--help]

  Sobe o mayara-server lab na porta 6502 com:
    mayara-server --emulator
  (compose: docker/docker-compose.lab.yml). Flags extras do emulator
  (lab-ship / stationary / posição do ativo 10002) entram no fork quando
  o código Mayara for estendido (RM-124 / docs/lps/FJ-mayara-radar.md).

Opções:
  --build       Rebuild a partir do Dockerfile deste clone (Rust/Alpine; lento).
                Necessário após mudar o código do emulator no fork.
  -h, --help    Mostra esta ajuda.

Comportamento:
  - Running + /signalk OK → sai (idempotente).
  - Sem imagem local → docker pull GHCR e tag mayara-server:local (rápido).
  - --build → docker compose up -d --build (só quando quiseres compilar o clone).
EOF
}

ensure_local_image() {
  if docker image inspect "$IMAGE_LOCAL" &>/dev/null; then
    return 0
  fi
  echo "==> imagem $IMAGE_LOCAL ausente — pull $IMAGE_GHCR …"
  docker pull "$IMAGE_GHCR" || die "pull $IMAGE_GHCR falhou"
  docker tag "$IMAGE_GHCR" "$IMAGE_LOCAL"
  echo "==> marcada como $IMAGE_LOCAL"
}

compose() {
  docker compose -f "$COMPOSE_FILE" "$@"
}

compose_fail_hint() {
  echo "erro: docker compose falhou." >&2
  echo "Se a mensagem for 'permission denied' no socket Docker, tente:" >&2
  echo "  sg docker -c './scripts/dev/up-local.sh'" >&2
  echo "ou adicione seu usuário ao grupo docker e reinicie a sessão." >&2
}

print_footer() {
  echo
  echo "container: $CONTAINER"
  echo "porta:     $PORT"
  echo "Signal K:  http://127.0.0.1:${PORT}/signalk"
  echo "Health:    $HEALTH_URL"
  echo "Logs:      docker logs -f $CONTAINER"
  echo "Parar:     ./scripts/dev/down-local.sh"
  echo
  echo "Auto-start: restart=unless-stopped — com docker.service enabled, o container volta ao abrir WSL."
  echo "Rebuild:    ./scripts/dev/up-local.sh --build"
  echo "Nota:       primeiro build Rust pode demorar vários minutos."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build)
      FORCE_BUILD=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "flag desconhecida: $1 (use -h)"
      ;;
  esac
done

[[ -f "$COMPOSE_FILE" ]] || die "compose lab não encontrado: $COMPOSE_FILE"

PORT="${MAYARA_HOST_PORT:-$DEFAULT_PORT}"
HEALTH_URL="http://127.0.0.1:${PORT}/signalk"

# Idempotente: já running + /signalk OK → sem compose up/build.
if docker inspect -f '{{.State.Status}}' "$CONTAINER" 2>/dev/null | grep -qx running; then
  if curl -sf "$HEALTH_URL" &>/dev/null; then
    if (( FORCE_BUILD )); then
      echo "==> container $CONTAINER já healthy — --build força recriar …"
    else
      echo "container $CONTAINER já está em execução e healthy (idempotente)."
      echo "Rebuild só com: ./scripts/dev/up-local.sh --build"
      print_footer
      exit 0
    fi
  fi
fi

if (( FORCE_BUILD )); then
  export DOCKER_BUILDKIT=1
  echo "==> docker compose up -d --build (Dockerfile do clone; Rust/Alpine; build network=host + IPv4 preferido; pode demorar) …"
  if ! compose up -d --build; then
    compose_fail_hint
    exit 1
  fi
else
  ensure_local_image
  echo "==> docker compose up -d (sem rebuild; --build só para compilar o clone) …"
  if ! compose up -d; then
    compose_fail_hint
    exit 1
  fi
fi

echo "==> Aguardando mayara /signalk (até ${HEALTH_TIMEOUT_SEC}s) …"
deadline=$((SECONDS + HEALTH_TIMEOUT_SEC))
ready=0
while (( SECONDS < deadline )); do
  if curl -sf "$HEALTH_URL" &>/dev/null; then
    ready=1
    break
  fi
  sleep 2
done

if (( ready )); then
  echo "mayara lab pronto."
else
  echo "aviso: timeout aguardando /signalk; o server pode ainda estar subindo." >&2
  echo "Logs: docker logs -f $CONTAINER" >&2
fi

print_footer

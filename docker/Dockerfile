# syntax=docker/dockerfile:1
FROM rust:1.94-alpine AS builder
# Prefer IPv4 *before* any network fetch: Alpine CDN / crates.io AAAA often
# stall in Docker/WSL (apk I/O error, Cargo "spurious network error").
RUN printf 'precedence :ffff:0:0/96  100\n' > /etc/gai.conf
# `curl` is needed at build time by utoipa-swagger-ui's build.rs to fetch
# the Swagger UI bundle; without it the build fails with `curl command not
# found`. `perl` is required by openssl-sys. `git` + CLI fetch: git deps in
# Cargo.toml (nmea-parser / tokio-tungstenite) respect gai.conf.
# Cache mount needs apk to write under /var/cache/apk — omit --no-cache.
RUN --mount=type=cache,id=mayara-apk,target=/var/cache/apk \
    apk add build-base curl perl cmake git
ENV CARGO_NET_GIT_FETCH_WITH_CLI=true

WORKDIR /usr/src/mayara-server

# BuildKit cache mounts keep crates.io / git deps and `target/` across rebuilds
# (even when Cargo.toml changes). Stable `id=` so prune type=regular does not
# drop them. The release binary is copied out of the `target` mount into the
# image layer so the final stage can COPY it.
COPY Cargo.lock Cargo.toml build.rs ./
RUN --mount=type=cache,id=mayara-cargo-registry,target=/usr/local/cargo/registry \
    --mount=type=cache,id=mayara-cargo-git,target=/usr/local/cargo/git \
    --mount=type=cache,id=mayara-cargo-target,target=/usr/src/mayara-server/target \
    mkdir src && echo "fn main() {}" > src/main.rs \
    && cargo build --release 2>/dev/null || true \
    && rm -rf src

COPY src src/
# Cargo parses the whole manifest (including `[[example]]` blocks) on every
# build. The generate-fixtures example is gated by `required-features =
# ["pcap-replay"]` and is never built here, but the manifest is invalid
# unless its source file is present.
COPY examples examples/
COPY web web/
RUN --mount=type=cache,id=mayara-cargo-registry,target=/usr/local/cargo/registry \
    --mount=type=cache,id=mayara-cargo-git,target=/usr/local/cargo/git \
    --mount=type=cache,id=mayara-cargo-target,target=/usr/src/mayara-server/target \
    # pcap-replay: lab FL pode subir `--pcap` (Furuno fixture em testdata/pcap).
    cargo build --release --features pcap-replay \
    && cp /usr/src/mayara-server/target/release/mayara-server /usr/src/mayara-server/mayara-server \
    && cp /usr/src/mayara-server/target/release/geo-echo-renderer /usr/src/mayara-server/geo-echo-renderer

FROM alpine:3.21
LABEL org.opencontainers.image.source="https://github.com/MarineYachtRadar/mayara-server"
LABEL org.opencontainers.image.title="mayara-server"
LABEL org.opencontainers.image.description="Marine radar server with REST API and WebSocket support"
LABEL org.opencontainers.image.license="Apache-2.0"
RUN printf 'precedence :ffff:0:0/96  100\n' > /etc/gai.conf
RUN --mount=type=cache,id=mayara-apk,target=/var/cache/apk \
    apk add tini \
    && adduser -D -h /home/mayara mayara
COPY --from=builder /usr/src/mayara-server/mayara-server /usr/local/bin/mayara-server
COPY --from=builder /usr/src/mayara-server/geo-echo-renderer /usr/local/bin/geo-echo-renderer
# Names this deployment in the usage stats a user may agree to send, so a
# container install is not counted as someone running the binary by hand.
# A packager that builds on this image overrides it (the Signal K plugin
# passes signalk-server-plugin).
ENV MAYARA_DEPLOYMENT=container
USER mayara
EXPOSE 6502 6503
# Probe the always-served, unauthenticated discovery doc so the container
# reports healthy once mayara's HTTP server is up. Deliberately checks only
# mayara itself, not the upstream Signal K connection — a boat offline from
# Signal K still runs a healthy radar server. Uses BusyBox wget (already in
# the Alpine base; no curl). The in-container port is always 6502 regardless
# of how Signal K proxies it externally.
# Note: geo-echo-renderer (CMD override) listens on 6503; its /health is not
# this HEALTHCHECK — lab compose may override or ignore when that binary runs.
HEALTHCHECK --interval=30s --timeout=5s --start-period=15s --retries=3 \
    CMD wget -q -O /dev/null http://127.0.0.1:6502/signalk || exit 1
ENTRYPOINT ["tini", "--"]
CMD ["mayara-server"]

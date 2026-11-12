# syntax=docker/dockerfile:1
# planner_dwa のイメージ。リポジトリのルートを context にしてビルドする:
#   docker build -f docker/Dockerfile -t planner_dwa .
#
# Rust のプラグインは C ABI なので、juiz は pip の juiz-cli をそのまま使う(版は Cargo.toml の juiz_sdk と揃える)。
# mobile_sim のイメージは juiz run ではなく juiz -f で動くので、こちらも conf の HTTP の subsystem でつなぐ。
ARG JUIZ_VERSION=0.2.7

# プラグインをビルドしてテストする
FROM rust:1-bookworm AS build
ARG JUIZ_VERSION
WORKDIR /src
COPY . .
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/src/target \
    grep -qx "juiz_sdk = \"=${JUIZ_VERSION}\"" Cargo.toml \
    && cargo test --workspace --locked \
    && cargo build -p planner_dwa --release --locked \
    && target=$(rustc -vV | sed -n 's/^host: //p') \
    && mkdir -p "/plugins/$target" \
    && cp target/release/libplanner_dwa.so "/plugins/$target/"

FROM python:3.12-slim-bookworm
ARG JUIZ_VERSION
RUN pip install --no-cache-dir "juiz-cli==${JUIZ_VERSION}"
# app/ と同じ形にする(conf からの相対パスでプラグインとブラウザ画面を探す)
COPY app/app.conf /opt/planner_dwa/
COPY app/web /opt/planner_dwa/web
COPY --from=build /plugins /opt/planner_dwa/plugins
COPY LICENSE-MIT LICENSE-APACHE /opt/planner_dwa/
COPY docker/entrypoint.sh /usr/local/bin/planner_dwa-entrypoint
ENV JUIZ_APP_DIR=/opt/planner_dwa
WORKDIR /opt/planner_dwa
# つなぐ mobile_sim(juiz の HTTP)
ENV MOBILE_SIM_URL=http://mobile_sim:8000
# 自己位置の元: amcl(localization_amcl と一緒に)か odom(mobile_sim だけで試す)
ENV DWA_POSE_SOURCE=amcl
# juiz#449 の回避(bridge)で読む localization_amcl と planner_dijkstra。空にすると読まない
ENV DWA_BRIDGE_AMCL_URL=http://localization_amcl:8002
ENV DWA_BRIDGE_PLANNER_URL=http://planner_dijkstra:8003
EXPOSE 8004
ENTRYPOINT ["planner_dwa-entrypoint"]

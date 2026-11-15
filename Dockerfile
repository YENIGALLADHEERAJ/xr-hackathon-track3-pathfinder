# Path Finder -- XR Hackathon 2026, Track 03
#
# Build:   docker build -t pathfinder .
# Run:     docker run --rm -v "$PWD/data:/data" pathfinder /data/evaluation_dataset.zip \
#                 -o /data/estimated_trajectory.txt --metadata /data/metadata.json
#
# The image contains the C++ build (primary, /usr/local/bin/pathfinder) and the
# byte-identical Python reference (/opt/pathfinder/main.py, stdlib only).

FROM debian:bookworm-slim AS build
RUN apt-get update \
 && apt-get install -y --no-install-recommends g++ \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY src/main.cpp .
RUN g++ -O2 -std=c++17 -Wall -Wextra -o pathfinder main.cpp

FROM debian:bookworm-slim
RUN apt-get update \
 && apt-get install -y --no-install-recommends python3 \
 && rm -rf /var/lib/apt/lists/*
COPY --from=build /src/pathfinder /usr/local/bin/pathfinder
COPY src/main.py /opt/pathfinder/main.py
WORKDIR /data
ENTRYPOINT ["pathfinder"]
CMD ["--help"]

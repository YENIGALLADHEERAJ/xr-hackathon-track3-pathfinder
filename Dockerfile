# Path Finder -- XR Hackathon 2026, Track 03 (Python submission, stdlib only)
#
# Build:   docker build -t pathfinder .
# Run:     docker run --rm -v "$PWD/data:/data" pathfinder /data/evaluation_dataset.zip \
#                 -o /data/estimated_trajectory.txt --metadata /data/metadata.json

FROM python:3.11-slim
COPY src/main.py /opt/pathfinder/main.py
WORKDIR /data
ENTRYPOINT ["python", "/opt/pathfinder/main.py"]
CMD ["--help"]

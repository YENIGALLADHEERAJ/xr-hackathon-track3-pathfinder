# Path Finder -- XR Hackathon 2026, Track 03 (Python submission, stdlib only)
#
# Build:   docker build -t pathfinder .
# Run:     put evaluation_dataset.zip in a folder named data, then from the folder above it:
#            docker run --rm -v "$PWD/data:/data" pathfinder evaluation_dataset.zip \
#                   -o estimated_trajectory.txt --metadata metadata.json
#          Paths are relative to /data (the working directory). With no arguments the
#          container finds imu0/data.csv under /data and writes estimated_trajectory.txt there.

FROM python:3.11-slim

LABEL org.opencontainers.image.title="pathfinder" \
      org.opencontainers.image.description="XR Hackathon 2026 Track 03: IMU-only 6-DoF trajectory estimation" \
      org.opencontainers.image.source="https://github.com/YENIGALLADHEERAJ/xr-hackathon-track3-pathfinder"

# no .pyc files in the image or the mounted folder; logs appear immediately
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

COPY src/main.py /opt/pathfinder/main.py

# fail the build if the script does not even start
RUN python /opt/pathfinder/main.py --help > /dev/null

WORKDIR /data
ENTRYPOINT ["python", "/opt/pathfinder/main.py"]

#!/bin/bash
# run_s3li_evaluation.sh - Main script to build and run S3LI evaluation

set -e

echo "=========================================="
echo "DLR S3LI Dataset SLAM Evaluation"
echo "=========================================="

# Configuration
CONTAINER_NAME="s3li_slam_eval"
IMAGE_NAME="s3li_slam:latest"
DATASET_DIR="./dataset"
RESULTS_DIR="./results"

# Create necessary directories
mkdir -p $DATASET_DIR $RESULTS_DIR configs evaluation_scripts

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container $CONTAINER_NAME already exists."
    read -p "Do you want to restart it? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker stop $CONTAINER_NAME || true
        docker rm $CONTAINER_NAME || true
    else
        echo "Starting existing container..."
        docker start $CONTAINER_NAME
        echo "Container started. Access it with: docker exec -it $CONTAINER_NAME bash"
        exit 0
    fi
fi

# Build Docker image
echo "Building Docker image..."
docker build -t $IMAGE_NAME .

# Run container
echo "Starting container..."
docker run -d \
    --name $CONTAINER_NAME \
    --restart unless-stopped \
    -e DISPLAY=${DISPLAY} \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/$DATASET_DIR:/workspace/dataset \
    -v $(pwd)/$RESULTS_DIR:/workspace/results \
    -v $(pwd)/configs:/workspace/configs \
    -v $(pwd)/evaluation_scripts:/workspace/scripts \
    --shm-size=8gb \
    $IMAGE_NAME

# Wait for container to be ready
sleep 3

echo ""
echo "=========================================="
echo "Container started successfully!"
echo "=========================================="
echo ""
echo "Container Name: $CONTAINER_NAME"
echo "Container ID: $(docker ps -qf "name=$CONTAINER_NAME")"
echo ""
echo "To access the container:"
echo "  docker exec -it $CONTAINER_NAME bash"
echo ""
echo "To download dataset:"
echo "  docker exec -it $CONTAINER_NAME bash /workspace/scripts/download_dataset.sh"
echo ""
echo "To run evaluations:"
echo "  docker exec -it $CONTAINER_NAME bash /workspace/scripts/run_evaluation.sh"
echo ""
echo "To stop container:"
echo "  docker stop $CONTAINER_NAME"
echo ""
echo "To view logs:"
echo "  docker logs -f $CONTAINER_NAME"
echo ""

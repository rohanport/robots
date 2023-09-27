# Robots

Mono-repo for ROS robot simulations. Uses the [sea-bass ROS2 docker setup](https://github.com/sea-bass/turtlebot3_behavior_demos)

---

## Setup

First, install Docker and Docker Compose using [the official install guide](https://docs.docker.com/engine/install/ubuntu/).

To run Docker containers with NVIDIA GPU support, you can optionally install the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker).

First, clone this repository and go into the top-level folder:

```
git clone https://github.com/rohanport/robots.git
cd robots
```

Build the Docker images.
This will take a while and requires approximately 4 GB of disk space.

```
docker compose build
```

---

## Basic Usage

We use [Docker Compose](https://docs.docker.com/compose/) to automate building, as shown above, but also for various useful entry points into the Docker container once it has been built.
**All `docker compose` commands below should be run from your host machine, and not from inside the container**.

To enter a Terminal in the overlay container, first start a container:

```
docker compose up overlay
```

Then, in a separate Terminal, you can access the running container:

```
docker exec -it robots-overlay-1 bash
```

If you want to develop using Docker, you can can also launch a dev container using:

```
# Start the dev container
docker compose up dev

# Open as many interactive shells as you want to the container
docker compose exec -it dev bash
```

# docker for RPi robots

See Kev's Robots for reference: https://www.kevsrobots.com/learn/learn_ros/07_build_container.html

Build and run from this directory.

## Building

```
docker build -t ros2 .
```

## Running

Note that you may want to tweak the `docker-compose.yml` file before you run it. In particular, you may need to update the devices you share with docker (such as I2C device path at the very least).

```
docker compose up -d
```

## Connect to running container

```
docker exec -it docker-full_ros2_1 bash
```

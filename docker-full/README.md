# docker for RPi robots

See Kev's Robots for reference: https://www.kevsrobots.com/learn/learn_ros/07_build_container.html

Build and run from this directory.

Building:
```
docker build -t ros2 .
```

Running:
```
docker-compose up -d
```

Connect to running container:
```
docker exec -it docker-full_ros2_1 bash
```
docker rm -f mobed-container 2>/dev/null; \
docker run --rm -it \
  --name mobed-container \
  -v /Users/ricardo/Codes/MobED:/workspace \
  ros2-dev \
  /bin/bash
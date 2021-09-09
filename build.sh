docker build \
  -f Dockerfile \
  -t dependency_injection_ros:latest \
  --build-arg USER=$USER \
  --build-arg UID=$(id -u) \
  --build-arg GID=$(id -g) \
  .

docker run -it \
  --hostname="di-ros" \
  --add-host="di-ros:127.0.0.1" \
  --user=$(id -u $USER):$(id -g $USER)        \
  --workdir="/home/$USER/ws"            \
  --volume="$PWD:/home/$USER/ws/src"        \
  --volume="$HOME/.gitconfig:$HOME/.gitconfig"        \
  --volume="/etc/group:/etc/group:ro"         \
  --volume="/etc/passwd:/etc/passwd:ro"       \
  --volume="/etc/shadow:/etc/shadow:ro"       \
  --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
  --net=host \
  dependency_injection_ros:latest \
  bash

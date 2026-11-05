#!/bin/bash
# Entry point of the caging simulator container: load ROS 2 and the workspace, then run
# the command given to the container (bash by default).
set -e
# The gui service adds the host's video/render group IDs; give a name to any ID the image
# does not know, otherwise bash prints "groups: cannot find name for group ID ...".
for gid in $(id -G); do
  getent group "${gid}" >/dev/null || sudo groupadd --gid "${gid}" "host${gid}"
done
source /opt/ros/${ROS_DISTRO}/setup.bash
# In the development service src/ is mounted from the host and built into its own install/ folder;
# use it once it exists, otherwise the image's build.
if [ -f "${WS}/install/setup.bash" ]; then
  source "${WS}/install/setup.bash"
fi
exec "$@"

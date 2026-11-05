#!/bin/bash
# ==================================================================================================
# Build the student image and upload it to the GitHub container registry.
# Run by the author, from the repository root:
#
#   bash docker/publish.sh            # build, then push (asks before pushing)
#   bash docker/publish.sh --no-push  # only build and tag (to test the student kit locally)
#
# Once per computer before the first push: a GitHub personal access token (classic) with the
# scope write:packages, then
#   echo <token> | docker login ghcr.io -u ddrozo31 --password-stdin
# After the very first push, make the package public once on GitHub:
#   github.com/ddrozo31 -> Packages -> caging_multirobot_ros2 -> Package settings -> Change visibility -> Public
# ==================================================================================================
set -e
REGISTRY_IMAGE=ghcr.io/ddrozo31/caging_multirobot_ros2
VERSION=v1.0                       # the frozen simulator; src/ must be identical to this tag
DATE=$(date +%Y%m%d)

# 1. only publish the frozen simulator, with the docker files committed
if ! git diff --quiet "${VERSION}" -- src; then
  echo "src/ differs from tag ${VERSION}: the student image must be the frozen simulator. Stop."; exit 1
fi
REVISION=$(git rev-parse --short HEAD)
DIRTY=$(git status --porcelain -- src docker .dockerignore)   # pushing needs a clean tree (below)

# 2. build from a fresh base image, with UID/GID 1000 (the first user on Linux and on WSL2)
docker build --pull -f docker/Dockerfile \
  --build-arg USER_UID=1000 --build-arg USER_GID=1000 \
  --build-arg VERSION="${VERSION}" --build-arg REVISION="${REVISION}" \
  -t "${REGISTRY_IMAGE}:jazzy" -t "${REGISTRY_IMAGE}:${VERSION}" -t "${REGISTRY_IMAGE}:${DATE}" .
echo "built ${REGISTRY_IMAGE} (tags jazzy, ${VERSION}, ${DATE}; commit ${REVISION})"

# 3. upload (about 3 GB compressed the first time; later only changed layers)
if [ "$1" = "--no-push" ]; then exit 0; fi
if [ -n "${DIRTY}" ]; then
  echo "Uncommitted changes in src/ or docker/: commit them first, so the image matches a commit. Not pushed."; exit 1
fi
read -r -p "Push the three tags to ghcr.io now? [y/N] " answer
if [ "${answer}" = "y" ]; then
  for tag in jazzy "${VERSION}" "${DATE}"; do docker push "${REGISTRY_IMAGE}:${tag}"; done
fi

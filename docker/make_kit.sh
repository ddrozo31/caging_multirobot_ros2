#!/bin/bash
# ==================================================================================================
# Make the student kit: a small zip (course page, GitHub release) with everything a student needs
# to run the published image, without cloning this repository.
#
#   bash docker/make_kit.sh     # from the repository root -> docker/dist/caging-docker-kit.zip
#
# Contents: compose.yaml (pull-only, docker/kit/), env.sh, README.md (= docker/kit/STUDENT_GUIDE.md),
# an empty results/ folder for the experiments.
# ==================================================================================================
set -e
DIST=docker/dist
KIT=${DIST}/caging-docker-kit
rm -rf "${KIT:?}" "${DIST:?}/caging-docker-kit.zip"
mkdir -p "${KIT}/results"
cp docker/kit/compose.yaml "${KIT}/compose.yaml"
cp docker/env.sh            "${KIT}/env.sh"
cp docker/kit/STUDENT_GUIDE.md "${KIT}/README.md"
touch "${KIT}/results/.keep"
(cd "${DIST}" && zip -qr caging-docker-kit.zip caging-docker-kit)
echo "written ${DIST}/caging-docker-kit.zip"; unzip -l "${DIST}/caging-docker-kit.zip"

#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 MattEdm
# SPDX-License-Identifier: GPL-3.0-or-later
# Builds both boards and puts the update files in release/:
#   manipulator.bin  (this project)    controller.bin  (../controller)
# Commit and tag the repository first (e.g. git tag v1.2): the version an update compares is git
# describe's, so a release built from a tagged, clean tree reads "v1.2" for both boards, and
# "-dirty" means uncommitted changes went into it.
#
# Then attach both files to a GitHub release, e.g.
#   gh release create v1.2 release/manipulator.bin release/controller.bin --title v1.2
# The menu's "Update firmware" fetches them from CONFIG_OM_UPDATE_URL (.../releases/latest/download).
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
controller="${CONTROLLER_DIR:-$here/../controller}"
source ~/.espressif/tools/activate_idf_v6.1.sh >/dev/null

(cd "$controller" && idf.py build >/dev/null)
(cd "$here" && idf.py build >/dev/null)

mkdir -p "$here/release"
cp "$here/build/omachine-manipulator.bin" "$here/release/manipulator.bin"
cp "$controller/build/omachine-controller.bin" "$here/release/controller.bin"
echo "manipulator: $(git -C "$here" describe --always --tags --dirty)"
echo "controller:  $(git -C "$controller" describe --always --tags --dirty)"
ls -l "$here/release"

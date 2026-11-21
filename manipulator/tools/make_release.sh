#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 MattEdm
# SPDX-License-Identifier: GPL-3.0-or-later
# Builds both boards and puts the update files in release/ (with ESP-IDF v6.1 activated):
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
# ESP-IDF's activate script won't run from inside another script (it takes this script's name to
# mean it was executed rather than sourced), so the environment has to be active already. Its
# idf.py is a shell function, which scripts don't inherit, so call the tool itself.
if [ -z "${IDF_PATH:-}" ]; then
    echo "Activate ESP-IDF first: source ~/.espressif/tools/activate_idf_v6.1.sh" >&2
    exit 1
fi
idf() { python "$IDF_PATH/tools/idf.py" "$@"; }

# Reconfigure first: the version is read from git when a project is configured, not on every build,
# so without it a new tag wouldn't reach the images
(cd "$controller" && idf reconfigure >/dev/null && idf build >/dev/null)
(cd "$here" && idf reconfigure >/dev/null && idf build >/dev/null)

mkdir -p "$here/release"
cp "$here/build/omachine-manipulator.bin" "$here/release/manipulator.bin"
cp "$controller/build/omachine-controller.bin" "$here/release/controller.bin"
echo "manipulator: $(git -C "$here" describe --always --tags --dirty)"
echo "controller:  $(git -C "$controller" describe --always --tags --dirty)"
ls -l "$here/release"

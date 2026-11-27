#!/usr/bin/env bash
set -eo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
cd "$root"
output="$root/releases"
if [[ $# -ne 0 ]]; then
    if [[ $# -eq 2 && $1 == --output-dir && -n $2 ]]; then
        output=$2
    else
        printf 'Usage: %s [--output-dir DIRECTORY]\n' "$0" >&2
        [[ $# -eq 1 && $1 == --help ]] && exit 0
        exit 2
    fi
fi
if [[ -n $(git status --porcelain) ]]; then
    printf '%s\n' 'Commit or isolate source changes before staging a field candidate.' >&2
    exit 2
fi
for package in slicer_interfaces slicer_base slicer_localization slicer_perception slicer_navigation slicer_gnss slicer_bringup slicer_operator_ui; do
    test -f "install/share/$package/package.xml" || {
        printf 'Build a complete install first. Missing %s\n' "$package" >&2
        exit 2
    }
done
test -f install/share/slicer_base/c30d/rtk/c30d_link.py
test -x install/lib/slicer_operator_ui/operator_ui.py
test -x install/lib/fast_lio/fastlio_mapping
test -x install/lib/livox_ros_driver2/livox_ros_driver2_node
revision=$(git rev-parse HEAD)
if [[ ! -f install/.slicer-candidate-revision || $(<install/.slicer-candidate-revision) != "$revision" ]]; then
    printf '%s\n' 'Build the selected commit before staging it.' >&2
    exit 2
fi
mkdir -p -- "$output"
output=$(cd -- "$output" && pwd -P)
candidate="$output/$revision"
if [[ -e "$candidate" ]]; then
    printf 'Candidate already staged at %s\n' "$candidate" >&2
    exit 2
fi
stage=$(mktemp -d "$output/.staging-XXXXXXXX")
trap 'rm -rf -- "$stage"' EXIT
cp -a install "$stage/install"
cp -a docs "$stage/docs"
install -m 0755 tools/run_field.sh "$stage/run"
cp -a tools/field_services "$stage/service"
/usr/bin/python3 - "$stage" "$revision" <<'PY'
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import yaml

stage, revision = Path(sys.argv[1]), sys.argv[2]
spec = yaml.safe_load(Path("dependencies.repos").read_text())["repositories"]
hardware_repo = "deps/slicer_ugv_hardware"
hardware_revision = spec[hardware_repo]["version"]
firmware_dir = stage / "firmware/c30d-v1.10"
firmware_dir.mkdir(parents=True)
for name in ("firmware.bin", "manifest.json"):
    artifact = subprocess.check_output([
        "git", "-C", hardware_repo, "show", f"{hardware_revision}:Firmware/v1.10/bin/{name}",
    ])
    (firmware_dir / name).write_bytes(artifact)

configs = {}
for path in sorted((stage / "install/share").glob("slicer_*/config/*")):
    if path.is_file():
        configs[str(path.relative_to(stage / "install"))] = hashlib.sha256(path.read_bytes()).hexdigest()
record = {
    "source_revision": revision,
    "dependencies": spec,
    "c30d_firmware_manifest": "firmware/c30d-v1.10/manifest.json",
    "installed_config_sha256": configs,
    "staged_at_utc": datetime.now(timezone.utc).isoformat(),
    "release_class": "field candidate",
}
(stage / "release.json").write_text(json.dumps(record, indent=2) + "\n")
PY
mv -T -- "$stage" "$candidate"
printf '%s\n' "$candidate"

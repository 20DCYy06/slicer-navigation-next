#!/usr/bin/env bash
set -eo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
services="$workspace/current/service"
configuration="$HOME/.config/slicer/services"
units="$HOME/.config/systemd/user"

if [[ ! -x "$services/run_unit" || ! -x "$services/ctl" ]]; then
    printf '%s\n' 'Select a sealed service-capable release as current first.' >&2
    exit 1
fi
if [[ $(loginctl show-user "$(id -u)" -p Linger --value) != yes ]]; then
    printf 'Enable user boot services with loginctl enable-linger %s, then rerun this installer.\n' "$(id -un)" >&2
    exit 1
fi
if [[ -L "$configuration/workspace" && $(realpath "$configuration/workspace") != "$workspace" ]]; then
    printf '%s\n' 'Field services are already bound to another workspace.' >&2
    exit 1
fi

install -d "$configuration" "$units"
if [[ ! -L "$configuration/workspace" ]]; then
    ln -s -- "$workspace" "$configuration/workspace"
fi
install -m 0644 "$services"/slicer-field-*.service "$units/"
systemctl --user daemon-reload
systemctl --user enable slicer-field-master.service slicer-field-base.service
printf '%s\n' 'Boot units installed. Start now with ./current/run service start master.'

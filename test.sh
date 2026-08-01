#!/bin/sh
# SPDX-FileCopyrightText: GARDENA GmbH
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Runs the flasher against the emulated SWD target.

set -eu

flasher=${1:?usage: $0 <binary>}
out=$(dirname "$flasher")
log=$out/test.log
fw=$out/fw.bin

"$flasher" -d | tee "$log"
grep -q 'IDCODE: 0x2ba01477' "$log"

head -c 4096 /dev/urandom > "$fw"
"$flasher" "$fw" | tee "$log"
grep -q 'Verification passed' "$log"

# The target comes up unerased, so verifying without writing must fail
if "$flasher" -v "$fw" > "$log" 2>&1; then
	echo "verify passed against flash that was never written" >&2
	exit 1
fi

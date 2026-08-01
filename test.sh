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

"$flasher" -d | tee "$log"
grep -q 'IDCODE: 0x2ba01477' "$log"

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

# Runs a command that has to fail, and checks why: a non-zero exit on its
# own passes for the wrong reason too easily, since almost any breakage
# reaches one eventually.
expect_fail()
{
	want=$1
	shift
	if "$@" > "$log" 2>&1; then
		echo "should have failed: $*" >&2
		exit 1
	fi
	if ! grep -q "$want" "$log"; then
		echo "wrong failure, expected: $want" >&2
		cat "$log" >&2
		exit 1
	fi
}

"$flasher" -d | tee "$log"
grep -q 'IDCODE: 0x2ba01477' "$log"

# An odd size leaves a trailing byte, padded to a halfword with 0xFF
head -c 4095 /dev/urandom > "$fw"
"$flasher" "$fw" | tee "$log"
grep -q 'Verification passed' "$log"

# A WAIT is backpressure, so the retry loop has to ride it out and still
# arrive at the same flash contents
SWD_MOCK_WAIT_EVERY=3 "$flasher" "$fw" > "$log"
grep -q 'Verification passed' "$log"

"$flasher" -e "$fw" > "$log"
grep -q 'Erase complete' "$log"
! grep -q 'Write complete' "$log"

# A FAULT is not backpressure and must not be retried away.  The rest have
# to fail too: a foreign part, flash nothing was written to, both exclusive
# options at once, and a file that is not there.
expect_fail 'ack=4' env SWD_MOCK_FAULT_AT=200 "$flasher" "$fw"
expect_fail 'Unexpected IDCODE' env SWD_MOCK_IDCODE=0x12345678 "$flasher" -d
expect_fail 'Verify mismatch' "$flasher" -v "$fw"
expect_fail 'mutually exclusive' "$flasher" -e -v "$fw"
expect_fail 'No such file' "$flasher" "$out/no-such-file.bin"

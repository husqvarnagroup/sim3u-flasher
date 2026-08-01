<!--
SPDX-FileCopyrightText: GARDENA GmbH

SPDX-License-Identifier: GPL-2.0-or-later
-->

# sim3u-flasher

Bit-banged SWD flasher for the Silicon Labs SiM3U167, driven from a Linux
host's GPIO. Built for GARDENA smart gateways (MT7688) and AT91SAM9X5-based
hosts; the host SoC is auto-detected from the device tree.

## Build

```
make
```

Cross-compile with `CROSS_COMPILE=<prefix>- make`.

## Usage

```
sim3u-flasher [options] <firmware.bin>
```

| Option | Effect |
|--------|--------|
| `-e` | erase only, do not write |
| `-v` | verify only, do not write |
| `-n` | skip verify after write |
| `-k` | keep CPU halted after done |
| `-d` | debug: dump IDCODE and exit |

`-e` and `-v` are mutually exclusive.

## Environment

| Variable | Meaning |
|----------|---------|
| `SWD_DELAY=N` | extra dummy register reads per clock half-period (default 0) |
| `SWD_SOC=NAME` | host SoC: `mt7688` or `at91sam9x5` (default: auto-detect) |
| `SWD_SWCLK=PIN` | SWCLK pin, e.g. `PC12` or `36` (default: per SoC) |
| `SWD_SWDIO=PIN` | SWDIO pin, e.g. `PC11` or `29` (default: per SoC) |

## Code style

| Target | Effect |
|--------|--------|
| `make format` | reformat the sources in place |
| `make format-check` | fail if anything is misformatted, changing nothing |
| `make lint` | run clang-tidy |
| `make check` | both checks, without writing — this is what CI runs |

Config is picked up automatically from `.clang-format` and `.clang-tidy`
in this directory.

## Tests

`make test` runs the flasher against an emulated SWD target, with no
hardware attached.

`./test-on-gateway.sh <gateway>` covers what the emulator cannot: the
`/dev/mem` mapping, the host GPIO backends and the real target's timing.

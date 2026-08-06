# PSS verification environment for the WISHBONE DMA/Bridge

The device's PSS model is `src/pss` (`wb_dma_c`, `wb_dma_ch_c`). This tree is
everything that is true of *this environment* rather than of the device, and it
is the layer a different integration would replace.

```
tests/pss/
  wb_dma_hs_pkg.pss       foreign-function contract of the handshake model
  wb_dma_hs_c(.pss|/)     hardware-handshake model, one per channel
  pss_top(.pss|/)         system assembly + the §5.2 sequences as scenarios
```

The device-plus-environment order is derived where it is needed, never checked
in. There was a `files.f` here; it went stale when the model was renamed, which
is what a snapshot of a derivation does.

```sh
pss-order . --include 'src/pss/**/*.pss' --include 'tests/pss/**/*.pss' \
            --exclude 'src/pss/wb_dma_cfg_pkg/wb_dma_cfg_nonblocking.pss' \
            --root pss_top --profile total-topological
```

The `--exclude` picks a build profile: `src/pss/wb_dma_cfg_pkg/` holds two
mutually-exclusive files declaring the same package, and exactly one may be
selected. `pss-order --from-flow src/pss --task src` reads that choice from the
build instead of restating it, but only for the device tree -- spanning both
trees has no task to read.

Note the single `base` spanning both trees: `src/pss/` and `tests/pss/` are
grouping directories, not units, so the two are interleaved by dependency
rather than concatenated. `wb_dma_hs_pkg` lands among the device packages
because that is where it belongs in the order, not where it lives on disk.

## Why these two are environment, not device

**`pss_top`** carries the integration constants -- where the register file sits
in the address map, and where system RAM is. Only the system knows those; the
register *offsets* and the channel stride stay in `wb_dma_regs_pkg` because
those are device knowledge. It also holds the §5.2 scenarios, which compose
device operations into tests.

**`wb_dma_hs_c`** models the hardware handshake, and the participant on that
interface is the PERIPHERAL DEVICE, not the DMA and not the processor. In a
real system these instances would belong to the requesting IP's own operation
model. Its platform requirement -- `wb_dma_hs_pkg`'s five foreign functions --
is satisfied by the environment, which is the second reason it does not ship
with the device.

## Checking

File order is load-bearing (`src/pss/README.md`, defect 3), so derive it rather
than glob:

```sh
pssparser $(pss-order . --include 'src/pss/**/*.pss' \
    --include 'tests/pss/**/*.pss' \
    --exclude 'src/pss/wb_dma_cfg_pkg/wb_dma_cfg_nonblocking.pss' \
    --root pss_top)
```

This still prints two `TaskResolveSymbolPathRef: Failed to get scope` lines
from the front end -- internal errors rather than diagnostics; the run itself
ends `0 errors` and exits 0. They predate the device/environment split and come
from `pss_top.pss`'s `init_down`; see D2 in
[`docs/pssparser-defects-2026-08-02.md`](../../docs/pssparser-defects-2026-08-02.md).
The device on its own (`pssparser $(pss-order --from-flow src/pss --task src
--relative-to .)`) is entirely silent.

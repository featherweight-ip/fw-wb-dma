# PSS implementation of the WISHBONE DMA/Bridge operation model

Source for the operation model in `docs/wb_dma_operation_model.md`, against the
register map in `docs/dma_doc.md` (Rev. 1.5) and the behaviour already modelled
in `src/spl/`.

Layout follows `skills/pss-coding-guidelines`: one element per file, components
as a top-level file plus a same-named directory, everything else attached with
`extend` under `functions/` or `actions/`.

## Where the operations live

| Operation (model §) | Level | File |
|---|---|---|
| `configure_channel` §3.2 | configuration | `wb_dma_ch_ops_c/functions/configure_channel.pss` |
| `transfer_single` §3.2 (was `run_transfer`) | end-to-end | `wb_dma_ch_ops_c/functions/transfer_single.pss` |
| `stop_channel` §3.2 | end-to-end | `wb_dma_ch_ops_c/functions/stop_channel.pss` |
| `set_auto_restart` §3.2 | configuration | `wb_dma_ch_ops_c/functions/set_auto_restart.pss` |
| `set_software_pointer` §3.2 | configuration | `wb_dma_ch_ops_c/functions/set_software_pointer.pss` |
| `transfer_list` §3.2 (was `run_descriptor_chain`) | end-to-end | `wb_dma_ch_ops_c/functions/transfer_list.pss` |
| `pause_engine` §3.2 | configuration | `wb_dma_ops_c/functions/pause_engine.pss` |
| `configure_interrupt_routing` §3.2 | configuration | `wb_dma_ops_c/functions/configure_interrupt_routing.pss` |
| `write_descriptor` §3.2 (was `write_descriptor_list`) | configuration | `wb_dma_ops_c/functions/write_descriptor.pss` |
| `read_descriptor_residual` §3.2 | configuration | `wb_dma_ops_c/functions/read_descriptor_residual.pss` |
| `request_chunk` §4 | end-to-end | `wb_dma_hs_c/functions/request_chunk.pss` |
| `skip_to_next_descriptor` §4 | end-to-end | `wb_dma_hs_c/functions/skip_to_next_descriptor.pss` |
| `restart_transfer` §4 | configuration | `wb_dma_hs_c/functions/restart_transfer.pss` |

`wait_completion` is not an operation; it is the shared wait point of every
end-to-end MMIO operation, and the file carries the reasoning about which
register a repeated `check_cond` may safely read.

`bridge_access` is omitted by review decision.

## Review decisions carried in from the model document

- `run_transfer` / `run_descriptor_chain` renamed to `transfer_single` /
  `transfer_list` for name symmetry.
- `write_descriptor_list` reshaped to take `at` and `prev` and return the next
  slot, so a caller can build a list in place one link at a time.
- Operations are per-channel, and a waking thread reads its own channel's
  status register. That is why `wb_dma_ch_ops_c` is a component array rather
  than a `chan` parameter on every function -- it keeps register access
  index-free.
- `bridge_access` omitted.

## Structure

```
src/pss/
  wb_dma_types_pkg.pss    shared data types: enums, channel config, descriptor
  wb_dma_hs_pkg.pss       the handshake model's foreign-function contract
  wb_dma_regs_pkg/
    wb_dma_regs_c.pss     address map + engine-global bank
    wb_dma_ch_regs_c.pss  one channel's bank
  wb_dma_ops_c(.pss|/)    MMIO model: engine-global operations, owner of the
                          per-channel operation components
  wb_dma_ch_ops_c(.pss|/) MMIO model: everything per-channel
  wb_dma_hs_c(.pss|/)     hardware-handshake model, one per channel
  pss_top(.pss|/)         system assembly + the §5.2 sequences as scenarios
```

Granularity follows rule 1 of the coding guidelines: components, functions and
actions get their own files; data types and register banks are grouped, because
nothing ever includes one enum or one register without its neighbours.

**File order matters** (see defect 3 below). Dependency order is:

```
wb_dma_types_pkg.pss
wb_dma_regs_pkg/wb_dma_regs_c.pss
wb_dma_regs_pkg/wb_dma_ch_regs_c.pss
wb_dma_hs_pkg.pss
wb_dma_hs_c.pss  (+ its functions/ and actions/)
wb_dma_ch_ops_c.pss  (+ its functions/ and actions/)
wb_dma_ops_c.pss  (+ its functions/ and actions/)
pss_top.pss  (+ its actions/)
```

Each `wb_dma_ch_ops_c` owns a top-level `wb_dma_ch_regs_c` group bound to its
own non-allocatable region at `WB_DMA_CH_BASE + chan*WB_DMA_CH_STRIDE`, rather
than the channel banks being an array nested in one device-wide group. That is
what makes every register access in the operation code index-free.

## Checking this source

```sh
pssparser --syntax-only $(find src/pss -name '*.pss')      # clean
```

Full semantic checking currently trips over two `pssparser` front-end defects,
both reproducible in a few lines and neither caused by this model:

1. **Composite field access inside an `extend component` body does not
   resolve.** Affects locals and parameters alike, in both functions and
   `exec` bodies. Since the coding guidelines put every function and action in
   an `extend` file, this hits nearly all of this source.

   ```pss
   struct s1 { bit a; }
   component c { }
   extend component c { function void f() { s1 v; v.a = 1; } }
   // error: root ref-path element v is not a composite scope
   // the same body written inline in `component c { ... }` is accepted
   ```

2. **SIGSEGV on any reference into a component-array element when the design
   contains a register group holding a register.**

   ```pss
   import std_pkg::*;
   import addr_reg_pkg::*;
   struct r_s : packed_s<> { rand bit[32] a; }
   pure component r_c : reg_c<r_s, READWRITE, 32> {}
   pure component grp_c : reg_group_c {
       r_c r0;                                   // drop this   -> no crash
       function bit[64] get_offset_of_instance(string name) { return 0; }
       function bit[64] get_offset_of_instance_array(string n, int i) { return -1; }
   }
   component sub_c {
       grp_c regs;
       solve function void f(addr_handle_t h) { regs.set_handle(h); }
   }
   component top_c {
       sub_c ch[4];                              // make it scalar -> no crash
       solve function void g(addr_handle_t b) { ch[0].f(b); }   // drop -> no crash
   }
   ```

3. **File order is load-bearing, and getting it wrong is a SIGSEGV rather than
   an error.** A reference to a type declared in a later-listed file of the
   same package crashes the linker. A plain alphabetical `find`/glob of this
   tree therefore segfaults; the dependency order above does not. Worth
   fixing because "pass the files in the wrong order" is the normal state of
   affairs for a globbed file list.

With all three worked around -- extends inlined, component arrays expanded to
named instances, files in dependency order -- the whole model checks clean, so
the defects are the only thing standing between this source and a green
`pssparser` run.

Two further environment notes:

- The VS Code PSS extension reports `Cannot resolve import 'addr_reg_pkg'` and
  `Cannot resolve base type 'reg_c'` throughout. Its language server constructs
  `WorkspaceIndex` with no stdlib directory, so `StdlibLoader` falls back to a
  bundled stub that defines only a minimal `std_pkg`. The real stdlib ships in
  the same extension at `packages/zuspec-fe-pss/src/stdlib/`; it is simply never
  wired up. `pssparser` resolves these fine.
- This model targets the `zuspec-fe-pss` stdlib, which is narrower than the PSS
  3.0 LRM. Missing pieces that changed how this source is written:

  | Missing | Consequence |
  |---|---|
  | `reg_c.write_field` / `write_masked` | read-modify-writes are spelled out as `read()` / mutate / `write()` |
  | `read_struct` / `write_struct` | `write_descriptor` serializes the packed struct onto `write32` by hand |
  | `list<>`, varargs `error()` / `format()` | no list parameters; `message()` takes a bare string |
  | enum base types (`enum e : bit[1]`) | packed structs use plain `bit` fields where an enum would read better |

  `read_struct`/`write_struct` are not merely absent from the extension's copy
  of the stdlib -- they are commented out inside `pssparser`'s own embedded
  `addr_reg_pkg`, under `/* TODO: generic type */`, alongside `read_bytes` and
  `write_bytes`. `wb_dma_desc_s` is nevertheless declared as the packed memory
  layout, so enabling them turns the serialization block in `write_descriptor`
  into a single `write_struct(at, d)` with no other change.

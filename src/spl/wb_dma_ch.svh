// One DMA channel: its MMIO register bank plus the WORKING set the engine
// iterates on. The register bank is expressed with the fw-hdl register model --
// `wb_dma_ch_regs` is an fw_reg_block of typed `fw_reg`s, one per programmed
// register (CSR/SZ/ADR0/AM0/ADR1/AM1/DESC/SWPTR), so the host/hardware access
// contract (which bits software writes, which hardware drives, which clear on
// read) is declared once as masks rather than re-implemented in the bus decode.
//
// `wb_dma_ch` wraps that bank with the engine-private working state (current
// source/dest addresses, remaining word count) and the field accessors / status
// updates the engine uses. Field reads project the typed CSR/SZ structs; status
// writes go through the register model's masked hardware-update path. A plain
// class (no process, no ports): the register file owns an array of these and the
// engine mutates their working state.

// The per-channel register file: eight 32-bit registers, offsets auto-assign at
// stride 4 (0x00..0x1c, 0x20 byte span). Masks capture the access contract:
//   sw_wmask  -- bits the host may write (RW config + the WO STOP pulse + CH_EN)
//   hw_wmask  -- bits the engine drives (status + ROC int-source, plus CH_EN and
//                STOP which hardware auto-clears on done/err and on abort)
//   rclr_mask -- int-source bits cleared as a side effect of a host CSR read
class wb_dma_ch_regs extends fw_reg_block #(32);
    fw_reg #(wb_dma_csr_t) csr;
    fw_reg #(wb_dma_sz_t)  sz;
    fw_reg #(bit [31:0])   adr0, am0, adr1, am1, desc, swptr;

    // Each register self-registers with this block (the `this` parent argument),
    // so offsets auto-assign at the uniform stride in construction order:
    // csr 0x00, sz 0x04, adr0 0x08, am0 0x0c, adr1 0x10, am1 0x14, desc 0x18,
    // swptr 0x1c -> size() == 0x20.
    function new(string name);
        super.new(name);
        csr   = new("csr",   this, .sw_wmask(csr_sw_wmask()),
                                   .hw_wmask(csr_hw_wmask()),
                                   .rclr_mask(csr_rclr()));
        sz    = new("sz",    this);
        adr0  = new("adr0",  this);
        am0   = new("am0",   this, .reset(32'hffff_fffc));   // reset value (spec §4.4.4)
        adr1  = new("adr1",  this);
        am1   = new("am1",   this, .reset(32'hffff_fffc));
        desc  = new("desc",  this);
        swptr = new("swptr", this);
    endfunction

    static function wb_dma_csr_t csr_hw_wmask();
        return '{ busy:1, done:1, err:1,
                  int_chk_done:1, int_done:1, int_err:1,
                  ch_en:1, stop:1, default:'0 };
    endfunction
    static function wb_dma_csr_t csr_sw_wmask();
        return '{ ine_chk_done:1, ine_done:1, ine_err:1, rest_en:1, prio:'1,
                  stop:1, sz_wb:1, use_ed:1, ars:1, mode:1,
                  inc_src:1, inc_dst:1, src_sel:1, dst_sel:1, ch_en:1, default:'0 };
    endfunction
    static function wb_dma_csr_t csr_rclr();
        return '{ int_chk_done:1, int_done:1, int_err:1, default:'0 };
    endfunction
endclass

class wb_dma_ch;
    int unsigned id;

    // Per-channel capabilities (RTL chN_conf = {CBUF, ED, ARS, EN}). When a
    // capability is absent the corresponding control bit is ignored (spec App. A).
    bit cap_en, cap_ars, cap_ed, cap_cbuf;

    // A VIEW of this channel's MMIO register bank.
    //
    // Not owned. The bank belongs to the register MAP (`wb_dma_regs`), which
    // creates it and places it at 0x20 + id*0x20; this is a handle to that
    // region. The channel used to construct it and donate it upward, which made
    // creation run opposite to containment and left "who owns this bank"
    // genuinely ambiguous -- the channel by construction, the map by address.
    //
    // Ownership is singular, views are plural: the engine, the register file and
    // the UVM reference model all reach these same registers, and none of them
    // is more the owner than this object is.
    wb_dma_ch_regs regs;

    // Working state (engine-private, loaded from the registers on start / reload).
    // "Armed/loaded, not yet done/errored" is not tracked separately -- it IS the
    // CSR BUSY status bit (set in arm(), cleared on done/err), which arbitration
    // reads directly.
    bit [31:0]   w_src, w_dst;
    int unsigned w_rem;     // words remaining in this TOT_SZ

    // `regs` is handed in, not built here -- see the member's comment.
    function new(int unsigned id, wb_dma_ch_regs regs,
                 bit cap_en = 1, bit cap_ars = 1, bit cap_ed = 1, bit cap_cbuf = 1);
        this.id       = id;
        this.regs     = regs;
        this.cap_en   = cap_en;
        this.cap_ars  = cap_ars;
        this.cap_ed   = cap_ed;
        this.cap_cbuf = cap_cbuf;
        w_src = '0; w_dst = '0; w_rem = 0;
    endfunction

    // No per-field read accessors: a client reads the typed register directly --
    // `regs.csr.read().<field>` (a wb_dma_csr_t) / `regs.sz.read().<field>` -- and
    // gets the bits by name. The capability gating that is NOT a plain field read
    // (e.g. ars = cap_ars & CSR.ars) is applied at the use site. The engine's own
    // status/interrupt bits are still written through named masked-update helpers,
    // since those carry the per-bit hw_wmask the client should not have to restate.

    // --- status / interrupt-source writes (engine-owned bits) -----------------
    // Each goes through the register model's masked hardware-update path, so it
    // can only touch hw-owned bits; a concurrent host write to control bits is
    // never clobbered. TASKs because update_val() is a task in the register model.
    task set_busy(bit v);   regs.csr.update('{busy:v, default:'0}, '{busy:1'b1, default:'0}); endtask
    task set_done(bit v);   regs.csr.update('{done:v, default:'0}, '{done:1'b1, default:'0}); endtask
    task set_err(bit v);    regs.csr.update('{err:v,  default:'0}, '{err:1'b1,  default:'0}); endtask
    task clr_stop();        regs.csr.update('{default:'0}, '{stop:1'b1,  default:'0}); endtask
    task clr_en();          regs.csr.update('{default:'0}, '{ch_en:1'b1, default:'0}); endtask // HW clears CH_EN on done/err
    task set_int_err();     regs.csr.update('{int_err:1'b1,      default:'0}, '{int_err:1'b1,      default:'0}); endtask
    task set_int_done();    regs.csr.update('{int_done:1'b1,     default:'0}, '{int_done:1'b1,     default:'0}); endtask
    task set_int_chunk();   regs.csr.update('{int_chk_done:1'b1, default:'0}, '{int_chk_done:1'b1, default:'0}); endtask

    // Load the working set from the programmed registers -- used at start, on ARS
    // auto-restart, and on hardware restart (§3.10).
    function void load_working();
        w_src = regs.adr0.read();
        w_dst = regs.adr1.read();
        w_rem = regs.sz.read().tot_sz;
    endfunction

    // Arm the channel on a rising CH_EN (invoked from the CSR write-observer):
    // load the working set and post fresh status (busy=1, done=0, err=0). BUSY is
    // the armed/loaded signal arbitration reads.
    task arm();
        load_working();
        regs.csr.update('{busy:1'b1, default:'0}, '{busy:1'b1, done:1'b1, err:1'b1, default:'0});
    endtask

    // Masked single-word (4-byte) address advance. Only mask bits increment
    // (circular buffers, §3.4); a channel without CBUF support uses a full mask
    // (plain increment). Default mask FFFFFFFC => normal increment.
    function bit [31:0] adv(bit [31:0] a, bit [31:0] mask);
        bit [31:0] m = cap_cbuf ? mask : 32'hffff_ffff;
        return (a & ~m) | ((a + 32'd4) & m);
    endfunction
    function void advance_src(); w_src = adv(w_src, regs.am0.read()); endfunction
    function void advance_dst(); w_dst = adv(w_dst, regs.am1.read()); endfunction
endclass

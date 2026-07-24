// Register file: the host-facing front end of the DMA. It keeps its protocol-
// independent fw_mem_if host port (the CPU programs registers through it; on the
// bus this is reached via a Wishbone slave + std_to_wb at the boundary) but now
// BACKS that port with the fw-hdl register model: a `wb_dma_regs` register block
// (globals + a per-channel sub-block array) decodes a byte offset to the target
// register and applies the documented access semantics (sw/hw write masks,
// read-to-clear, the CH_EN-start hardware hook) declaratively. The engine
// (wb_dma_de) reads/mutates the channels' working state directly; this component
// does not run a process.
//
// One side effect resists a pure register: INT_SRC_A/B are masked snapshot views
// over the engine's pending-source vector (read-only; cleared by a channel-CSR
// read, as in the RTL), so they stay explicit bus-read logic here rather than
// stored registers.

typedef class wb_dma_rf;

// Hardware write-observer on a channel CSR (the register model's `fw_reg_wr_if`
// hook -- its canonical use is exactly "CH_EN was set -> start the engine"). It
// fires after a committed host write; a rising CH_EN arms the channel.
class wb_dma_ch_start_obs implements fw_reg_wr_if #(32);
    local wb_dma_rf m_rf;
    local int       m_idx;
    function new(wb_dma_rf rf, int idx); m_rf = rf; m_idx = idx; endfunction
    virtual task on_write(input bit [31:0] val, input bit [31:0] prev);
        m_rf.on_ch_csr_write(m_idx, val, prev);
    endtask
endclass

// The top register file: global registers + an array of per-channel sub-blocks
// at 0x20 base / 0x20 stride. The channel banks are the same `wb_dma_ch_regs`
// the engine-facing `wb_dma_ch` objects own, nested here so one offset decode
// spans the whole map.
class wb_dma_regs extends fw_reg_block #(32);
    fw_reg #(wb_dma_gcsr_t)   csr;                  // global CSR (PAUSE @ bit0)
    fw_reg #(wb_dma_intvec_t) int_msk_a, int_msk_b; // per-channel interrupt masks

    function new(string name, wb_dma_ch ch[]);
        super.new(name);
        // Globals self-register at the block cursor: csr 0x00, int_msk_a 0x04,
        // int_msk_b 0x08.
        csr       = new("csr",       this);
        int_msk_a = new("int_msk_a", this);
        int_msk_b = new("int_msk_b", this);
        // INT_SRC_A/B (0x0c/0x10) are derived read-only masked views over the
        // engine's pending-source vector (see wb_dma_rf.host_read); they are not
        // stored registers and are intentionally NOT part of the block.
        // The per-channel banks are owned by the wb_dma_ch objects, so they are
        // nested explicitly at their fixed base (0x20, stride 0x20).
        foreach (ch[i]) add_block(ch[i].regs, 32 + i*32);
    endfunction
endclass

class wb_dma_rf extends fw_component;
    int unsigned n_ch;
    wb_dma_ch    ch[];
    wb_dma_regs  regs;                 // the MMIO register file (register model)

    bit [30:0]   int_pending;          // engine-set per-channel pending source

    // Interrupt-change notification (the wb_dma_int_if seam): sinks are called
    // whenever the aggregate {intb,inta} level moves. Purely observational -- with
    // no sinks registered, signal_int() is a cheap no-op (existing TBs unaffected).
    wb_dma_int_if   m_int_sinks[$];
    local bit [1:0] m_last_int;        // {intb, inta} last published

    // Provided host port: the Wishbone access() seam redirects to host_access(),
    // which dispatches to the register-block host_write()/host_read() helpers.
    `FW_WB_PROTO_IMP(32, 32, wb_dma_rf, host);

    function new(string name, fw_component parent, int unsigned n_ch = 4);
        super.new(name, parent);
        this.n_ch = n_ch;
        ch = new[n_ch];
        foreach (ch[i]) ch[i] = new(i);
        regs = new("regs", ch);
        int_pending = '0;
    endfunction

    function void build();
        host = new(this);
        // Wire the CH_EN-start hardware hook onto every channel CSR.
        foreach (ch[i]) begin
            wb_dma_ch_start_obs obs = new(this, i);
            ch[i].regs.csr.add_wr(obs);
        end
    endfunction

    function bit  paused();   return regs.csr.read().pause;                           endfunction
    // Level interrupt outputs: any pending source enabled by the mask.
    function bit  inta();     return |(int_pending & regs.int_msk_a.read().ch);       endfunction
    function bit  intb();     return |(int_pending & regs.int_msk_b.read().ch);       endfunction
    // Engine hook: record a pending interrupt source for channel c.
    function void raise_int(int unsigned c);
        if (c < n_ch) int_pending[c] = 1'b1;
        signal_int();
    endfunction

    // Interrupt-change seam: register a sink, and publish the aggregate level to
    // all sinks whenever it changes. Called after every event that can move the
    // {intb,inta} level (raise, channel-CSR read-clear, mask write).
    function void add_int_sink(wb_dma_int_if s); m_int_sinks.push_back(s); endfunction
    function void signal_int();
        bit a = inta();
        bit b = intb();
        if ({b, a} !== m_last_int) begin
            m_last_int = {b, a};
            foreach (m_int_sinks[i]) m_int_sinks[i].int_changed(a, b);
        end
    endfunction

    // CH_EN-start hook, invoked by a channel CSR write-observer after a host
    // write commits. A rising CH_EN (re)arms the channel, ignored while paused /
    // already active / when the channel lacks EN capability (spec App. A).
    task on_ch_csr_write(int idx, bit [31:0] val, bit [31:0] prev);
        wb_dma_csr_t v = wb_dma_csr_t'(val);
        wb_dma_csr_t p = wb_dma_csr_t'(prev);
        wb_dma_ch    c = ch[idx];
        // v is the just-committed CSR (busy is hw-owned, preserved through the sw
        // write), so v.busy is the channel's current armed state -- no extra read.
        if (!p.ch_en && v.ch_en && c.cap_en && !v.busy && !paused())
            c.arm();
    endtask

    // ---- wb_proto_if provider implementation --------------------------------
    // access() dispatches to the register-block write/read helpers below (kept
    // intact so the synthesizable register model is unchanged -- only the bus
    // BOUNDARY moves from fw_mem_if read/write to the Wishbone access() seam).
    virtual task host_access(input  logic [31:0] adr, input logic [31:0] dat_w,
                             input  logic [3:0]  sel, input bit          we,
                             output logic [31:0] dat_r, output bit        err);
        automatic bit e;
        if (we) begin host_write(e, adr, dat_w, sel); dat_r = 32'h0; end
        else          host_read(dat_r, e, adr);
        err = e;
    endtask

    // The bus access routes through the register block (offset decode + register
    // semantics); only the derived INT_SRC views are special-cased.
    virtual task host_write(output bit err, input logic [31:0] addr,
                            input logic [31:0] data, input logic [3:0] strb);
        bit [11:0] off = addr[11:0];          // register file occupies < 4 KB
        err = 1'b0;
        if (off != 12'h0c && off != 12'h10)   // INT_SRC_A/B are read-only
            regs.write_val(off, data);
        signal_int();                          // mask writes can move the int level
    endtask

    virtual task host_read(output logic [31:0] data, output bit err,
                           input logic [31:0] addr);
        bit [11:0] off = addr[11:0];
        err = 1'b0; data = '0;
        case (off)
            // INT_SRC_A/B are READ-ONLY masked snapshots -- reading them does NOT
            // clear anything (faithful to the RTL: int_srca/int_srcb are wires, and
            // the source bits are cleared only by a channel-CSR read, below).
            12'h0c: data = {1'b0, int_pending & regs.int_msk_a.read().ch};
            12'h10: data = {1'b0, int_pending & regs.int_msk_b.read().ch};
            default: begin
                data = regs.read_val(off);        // sw bus read (register semantics)
                // RTL semantics (wb_dma_ch_rf: ch_csr_re clears int_src_r): reading
                // a channel's CSR clears THAT channel's pending interrupt source.
                // Channel banks are at 0x20 base / 0x20 stride; the CSR is the bank
                // base (off aligned to 0x20, off >= 0x20, within the channel range).
                if (off[4:0] == 5'h0 && off >= 12'h20 &&
                    off < 12'(12'h20 + n_ch*32)) begin
                    int unsigned idx = (off - 12'h20) >> 5;
                    int_pending[idx] = 1'b0;
                end
            end
        endcase
        signal_int();                          // a channel-CSR read-clear can deassert the line
    endtask
endclass

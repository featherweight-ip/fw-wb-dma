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
    // The per-channel banks. THIS object creates and owns them, because the
    // register map is one artifact with one root: a bank at 0x20 + i*0x20 is
    // part of the map by definition of the address decode. Consumers (the
    // channel objects, the engine, the reference model) hold views onto these.
    wb_dma_ch_regs            bank[];

    function new(string name, int unsigned n_ch);
        super.new(name);
        // Globals self-register at the block cursor: csr 0x00, int_msk_a 0x04,
        // int_msk_b 0x08.
        csr       = new("csr",       this);
        int_msk_a = new("int_msk_a", this);
        int_msk_b = new("int_msk_b", this);
        // INT_SRC_A/B (0x0c/0x10) are derived read-only masked views over the
        // engine's pending-source vector (see wb_dma_rf.host_read); they are not
        // stored registers and are intentionally NOT part of the block.
        //
        // The per-channel banks go in at their fixed base (0x20, stride 0x20).
        // Built here rather than donated upward by the channels: creation now
        // runs the same direction as containment, which is what makes ownership
        // answerable at all -- and it matches the shape fw-hdl's own reg_dma
        // reference model uses.
        bank = new[n_ch];
        foreach (bank[i]) begin
            bank[i] = new($sformatf("ch%0d", i));
            add_block(bank[i], 32 + i*32);
        end
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
        // The MAP first -- it owns the whole address space including the channel
        // banks -- then the channel objects, each handed a view of its region.
        regs = new("regs", n_ch);
        ch   = new[n_ch];
        foreach (ch[i]) ch[i] = new(i, regs.bank[i]);
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
    //
    // Row 7 (`int_pending_set`). Emitted with the vector BEFORE and AFTER, not
    // just the channel index, because the interesting case is the one where
    // nothing moves: a re-raise onto an already-pending source is invisible in
    // any value trace and is exactly what "the interrupt fired once and then
    // stopped" looks like from outside.
    function void raise_int(int unsigned c);
        bit [30:0] prev = int_pending;
        if (c < n_ch) int_pending[c] = 1'b1;
        `fw_ev_state(dbg_if(), FW_L_TXN, "rf.int_pending_set")
            `fw_subject(c)
            `fw_field_u8_as("ch", c)
            `fw_state_change(longint'(prev), longint'(int_pending), FW_ACTOR_HW, 0)
        `fw_ev_end
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
            // Row 9 (`int_level`): the observable pin edge. The two per-bank AND
            // terms ride along because "the line moved" and "WHICH bank moved it
            // and why" are the same question in practice, and recomputing the
            // terms offline needs the mask values at that instant -- which no
            // other record carries.
            `fw_ev_state(dbg_if(), FW_L_TXN, "rf.int_level")
                `fw_state_change(longint'(m_last_int), longint'({b, a}), FW_ACTOR_HW, 0)
                `fw_field_h32_as("pending", int_pending)
                `fw_field_h32_as("term_a", int_pending & regs.int_msk_a.read().ch)
                `fw_field_h32_as("term_b", int_pending & regs.int_msk_b.read().ch)
            `fw_ev_end
            m_last_int = {b, a};
            foreach (m_int_sinks[i]) m_int_sinks[i].int_changed(a, b);
        end else if (int_pending != 0) begin
            // Row 10 (`int_level_stable`): "why did the line NOT assert". The
            // most common real answer is a mask -- the source latched into the
            // bank the handler is not watching -- and until now that produced
            // no record at all, because nothing happened.
            //
            // Gated on a pending source deliberately. signal_int() runs after
            // EVERY host access, and "the line did not move, and nothing was
            // pending" needs no explanation; emitting it would bury the case
            // that does. A diagnostic that fires when there is nothing to
            // diagnose is how a diagnostic gets filtered out and then ignored.
            //
            // Loss reasons (1-based; 0 keeps its "this one won" meaning):
            //   1 masked_out -- pending in neither bank's mask
            //   2 already_up -- the level is asserted; this source adds nothing
            `fw_ev_decision(dbg_if(), FW_L_OP, "rf.int_level_stable")
                `fw_field_h32_as("pending", int_pending)
                `fw_field_h32_as("msk_a", regs.int_msk_a.read().ch)
                `fw_field_h32_as("msk_b", regs.int_msk_b.read().ch)
                `fw_field_u8_as("level", m_last_int)
                for (int unsigned i = 0; i < n_ch; i++) begin
                    if (int_pending[i]) begin
                        `fw_dec_cand(i, (m_last_int != 2'b00) ? 2 : 1)
                    end
                end
            `fw_ev_end
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
        if (!p.ch_en && v.ch_en && c.cap_en && !v.busy && !paused()) begin
            c.arm();
            // The transfer flow OPENS here and closes in the engine
            // (`de.xfer_done`), joined by the flow key rather than a handle --
            // the two live on different threads and, in RTL, in different
            // blocks. `arm` is the accepting actor in the §14.10 sense: it makes
            // the work eligible without serving it, so `begun - accepted` on the
            // join is the channel's arbitration latency, measured.
            //
            // The key is just the channel: only one transfer is live per channel
            // at a time, so channel plus ordering pairs them unambiguously and
            // no arm counter has to be invented -- which matters, because a
            // counter the RTL does not hold could never be reconstructed there.
            `fw_op_accept(dbg_if(), FW_L_TXN, "rf.xfer")
                `fw_subject(idx)
                `fw_op_flow(idx)
                `fw_field_u8_as("ch", idx)
                `fw_field_u32_as("words", c.regs.sz.read().tot_sz)
            `fw_ev_end
            `fw_ev_decision(dbg_if(), FW_L_TXN, "rf.arm")
                `fw_subject(idx)
                `fw_field_u8_as("ch", idx)
                `fw_dec_winner(idx)
                `fw_field_h32_as("src", c.regs.adr0.read())
                `fw_field_h32_as("dst", c.regs.adr1.read())
                `fw_field_u32_as("words", c.regs.sz.read().tot_sz)
                `fw_field_u32_as("chunk", c.regs.sz.read().chk_sz)
                `fw_field_u8_as("prio", v.prio)
                `fw_field_b_as("hs", v.mode)
            `fw_ev_end
        end else if (!p.ch_en && v.ch_en) begin
            // "I set CH_EN and nothing happened." A rising CH_EN that does NOT
            // arm is the single most common way this model appears broken, and
            // until now it was completely silent -- the write landed, the
            // register read back exactly what was written, and the channel just
            // never ran. The reason is knowable only here, at the instant the
            // decision is made, which is why it is emitted rather than inferred.
            //
            // Reasons, in the order the guard tests them (1-based so 0 keeps its
            // "this candidate won" meaning):
            //   1 no_en_cap    -- the channel lacks the EN capability (App. A)
            //   2 already_busy -- a transfer is still running on this channel
            //   3 paused       -- the global PAUSE bit is set
            `fw_ev_decision(dbg_if(), FW_L_VITAL, "rf.arm_refused")
                `fw_subject(idx)
                `fw_field_u8_as("ch", idx)
                `fw_dec_cand(idx, !c.cap_en ? 1 : (v.busy ? 2 : 3))
            `fw_ev_end
        end
    endtask

    // ---- wb_proto_if provider implementation --------------------------------
    // access() dispatches to the register-block write/read helpers below (kept
    // intact so the synthesizable register model is unchanged -- only the bus
    // BOUNDARY moves from fw_mem_if read/write to the Wishbone access() seam).
    virtual task host_access(input  logic [31:0] adr, input logic [31:0] dat_w,
                             input  logic [3:0]  sel, input bit          we,
                             output logic [31:0] dat_r, output bit        err);
        automatic bit e;
        // Row 1 (`rf.host_access`): the model's host boundary, as a span.
        // Answers the first question anyone asks of a device that appears dead
        // -- "did my write reach the model at all" -- which no register event
        // can, because a write to an unmapped offset changes no register and a
        // write that never arrived looks exactly the same.
        `fw_op_span(hop)
        `fw_op_begin(dbg_if(), FW_L_TXN, "rf.host_access", hop)
            `fw_field_h32(adr)
            `fw_field_b(we)
            `fw_field_h32(dat_w)
            `fw_field_h8(sel)
        `fw_op_opened
        if (we) begin host_write(e, adr, dat_w, sel); dat_r = 32'h0; end
        else          host_read(dat_r, e, adr);
        err = e;
        `fw_op_close(dbg_if(), hop)
            `fw_field_h32(dat_r)
            `fw_field_b_as("err", e)
            `fw_op_outcome(e ? FW_OUT_ERROR : FW_OUT_OK)
        `fw_op_end(hop)
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
                    int unsigned idx  = (off - 12'h20) >> 5;
                    bit [30:0]   prev = int_pending;
                    int_pending[idx] = 1'b0;
                    // Row 8 (`int_pending_clear`): the DEASSERT half, and the
                    // one that was previously undiagnosable. This clear is a
                    // side effect of an ordinary CSR read -- a status poll, a
                    // reference model, a waveform dump reading the register --
                    // so an interrupt can disappear because something merely
                    // LOOKED at it. Attributing that to the read is the whole
                    // value of the record; `by` exists so a second cause
                    // (reset) can be told apart from this one.
                    `fw_ev_state(dbg_if(), FW_L_TXN, "rf.int_pending_clear")
                        `fw_subject(idx)
                        `fw_field_u8_as("ch", idx)
                        `fw_field_u8_as("by", 0)         // 0 = csr_read, 1 = reset
                        `fw_state_change(longint'(prev), longint'(int_pending),
                                         FW_ACTOR_SW, 0)
                    `fw_ev_end
                end
            end
        endcase
        signal_int();                          // a channel-CSR read-clear can deassert the line
    endtask
endclass

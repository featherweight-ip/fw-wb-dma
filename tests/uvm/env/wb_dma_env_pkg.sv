// ======================================================================
// wb_dma_env_pkg -- the complete UVM + fw-hdl environment for the wb_dma SPL
// class model, in one package.
//
// This is the whole test-side stack: the fw-hdl "DUT" flavours, the always-on
// SPL reference + comparison currency, and the UVM env/scoreboard/sequences.
// The flavour-specific pieces (three model views + three env views) all live
// here as classes; each hvl/hdl top SELECTS its flavour (which model root it
// news, which env it factory-overrides in, which vifs it seats) -- see
// tb/hvl_top_spl*.sv and tb/hvl_top_rtl.sv.
//
// The contents form a single linear dependency chain; the includes below are in
// that topological order:
//
//   model    -- fw-hdl class "DUT": two fw_mem_if memories, an interrupt-cause
//               collector, per-channel HW-handshake devices, the abstract model
//               base, and the TLM (method-driven) engine flavour.
//   ref      -- the passive SPL reference used by the SPL<->RTL comparison
//               harness: the canonical master-transaction record, the wb_proto_if
//               taps, and the tapped reference model.
//   uvm      -- the register-access agent (fwvip-wb initiator over the model's
//               wb_proto_if seam), scoreboard, comparator + monitor adapters,
//               the env, and the register/CSR sequence helpers.
//   wb       -- the Wishbone signal-level flavour: engine inside wb_dma_spl driven
//               over transactors, GPIO-observed interrupts, the augmented envs.
//   rtl      -- the OpenCores wb_dma_top flavour: host register path reused from
//               the WB flavour, data memories on combinational SV RAMs.
//
// One class per .svh, no `ifndef guards (the project convention); the package is
// compiled once into the shared env SimLib. Scenario sequences/tests live in
// wb_dma_tests_pkg.
// ======================================================================
`include "wb_dma_spl_macros.svh"   // FW_WB_PROTO_IMP / FW_WB_DMA_IRQ_IMP / FW_WB_DMA_HS_IMP
`include "uvm_macros.svh"

package wb_dma_env_pkg;
    import uvm_pkg::*;
    import fw_hdl_pkg::*;
    import fw_std_pkg::*;
    import fw_proto_wb_pkg::*;      // wb_proto_if + xtor bridges + wb_mem_* adapters
    import wb_dma_spl_pkg::*;       // wb_dma engine + wb_dma_csr_t / _sz_t
    import fwvip_wb_pkg::*;         // fwvip_wb initiator/target/monitor agents
    import fwvip_gpio_pkg::*;       // fwvip_gpio_monitor + fwvip_gpio_transaction

    // -----------------------------------------------------------------------
    // Shared module-level declarations, visible to every include below.
    // -----------------------------------------------------------------------

    // The model's 32-bit Wishbone edge -- the abstract BFM type used both as the
    // engine register slave and as the data-memory access API. Matches the RTL's
    // boundary, so the UVM env drives all three DUT flavours through one seam.
    typedef wb_proto_if #(32, 32) wb_dma_mem_if_t;

    // Wall-clock seconds from /proc/uptime (monotonic since boot, ~10 ms
    // resolution; DPI-free). Used by the tops and the perf test to measure
    // wall-clock runtime -- the delta between two calls is elapsed real time.
    function automatic real wc_now();
        int  fd;
        real up;
        up = 0.0;
        fd = $fopen("/proc/uptime", "r");
        if (fd == 0) return 0.0;
        void'($fscanf(fd, "%f", up));
        $fclose(fd);
        return up;
    endfunction

    // ---- register map (cf. packages/wb_dma/rtl wb_dma_defines.v) --------------
    localparam logic [31:0] REG_COR      = 32'h00;   // global CSR (bit0 = PAUSE)
    localparam logic [31:0] REG_INT_MASKA = 32'h04;
    localparam logic [31:0] REG_INT_MASKB = 32'h08;
    localparam logic [31:0] REG_INT_SRCA  = 32'h0c;  // read-only snapshot (clear via CH_CSR read)
    localparam logic [31:0] REG_INT_SRCB  = 32'h10;

    function automatic logic [31:0] CH_CSR (int c); return 32'h20 + 32'h20*c; endfunction
    function automatic logic [31:0] CH_TXSZ(int c); return 32'h24 + 32'h20*c; endfunction
    function automatic logic [31:0] CH_ADR0(int c); return 32'h28 + 32'h20*c; endfunction
    function automatic logic [31:0] CH_AM0 (int c); return 32'h2c + 32'h20*c; endfunction
    function automatic logic [31:0] CH_ADR1(int c); return 32'h30 + 32'h20*c; endfunction
    function automatic logic [31:0] CH_AM1 (int c); return 32'h34 + 32'h20*c; endfunction

    // Build a channel CSR word (bit positions match the RTL/spec).
    function automatic logic [31:0] mk_csr(bit ch_en, bit src_sel, bit dst_sel,
                                           bit inc_src, bit inc_dst, bit mode_hs,
                                           bit ars, logic [2:0] prio,
                                           bit ine_done = 1, bit ine_chunk = 0,
                                           bit ine_err = 0);
        logic [31:0] v = '0;
        v[0]  = ch_en;   v[1]  = dst_sel; v[2]  = src_sel;
        v[3]  = inc_dst; v[4]  = inc_src; v[5]  = mode_hs; v[6] = ars;
        v[15:13] = prio;
        v[17] = ine_err; v[18] = ine_done; v[19] = ine_chunk;
        return v;
    endfunction

    // SZ word: chunk in [24:16] (9b), tot in [11:0] (12b).
    function automatic logic [31:0] mk_sz(int chunk, int tot);
        return ((chunk & 32'h0000_01ff) << 16) | (tot & 32'h0000_0fff);
    endfunction

    // ---- model: fw-hdl class "DUT" (no UVM dependency in these classes) -------
    `include "wb_dma_mem.svh"
    `include "wb_dma_irq_collector.svh"
    `include "wb_dma_hs_device.svh"
    `include "wb_dma_model_base.svh"
    `include "wb_dma_tlm_model.svh"

    // ---- ref: always-on passive SPL reference + comparison currency -----------
    `include "wb_dma_xact.svh"
    `include "wb_dma_xact_tap.svh"
    `include "wb_dma_ref_model.svh"

    // ---- uvm: register agent, scoreboard, comparator, env, sequences ----------
    `include "wb_dma_scoreboard.svh"
    `include "wb_dma_reg_tee.svh"
    `include "wb_dma_comparator.svh"
    `include "wb_dma_mon_adapter.svh"
    `include "wb_dma_reg_forward.svh"
    `include "wb_dma_env.svh"
    `include "wb_dma_base_seq.svh"

    // ---- wb: Wishbone signal-level flavour (engine inside wb_dma_spl) ----------
    `include "wb_dma_wb_model.svh"
    `include "wb_dma_int_gpio_sub.svh"
    `include "wb_dma_wb_env.svh"
    `include "wb_dma_wbxtor_env.svh"

    // ---- rtl: OpenCores wb_dma_top flavour (data memories on SV RAMs) ----------
    `include "wb_dma_ram_mem.svh"
    `include "wb_dma_rtl_model.svh"
endpackage

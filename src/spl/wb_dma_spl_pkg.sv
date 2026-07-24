// Behavioral (class-based) model of the WISHBONE DMA/Bridge engine -- the "spl"
// model. One package holds the whole class layer: the DMA-specific types, the two
// DMA APIs, and the model components (one .svh per class, included below). The
// signal-level module wrapper (wb_dma_spl, ports matching wb_dma_top) lives in a
// separate compilation unit (wb_dma_spl.sv).
//
// Bus boundary: the engine speaks the WISHBONE access API (wb_proto_if, from
// fw-proto-wb) directly -- the SAME interface-class the RTL's transactors present
// -- so the fw-hdl model and the RTL share one boundary, which the signal-level
// wrapper bridges to WB pins via the fw-proto-wb xtor bridges. (This is a
// deliberate move away from the earlier protocol-independent fw_mem_if edge, to
// make RTL-vs-fw-hdl comparison apples-to-apples at the Wishbone boundary.) The
// interrupt outputs are
// driven through fw.proto.gpio's ABSTRACT drive API (gpio_drive_if) the same way
// -- the model depends on that method-level interface-class, NOT on the GPIO
// transactor (the kit's xtor bridges compile but are never instanced here; the
// signal-level initiator is instanced only at the wb_dma_spl edge). There is no
// dedicated types package -- the few shared types are declared directly here
// (behavioral model, so no synthesizable-sharing constraint).
//
// Package dependency: fw_hdl_pkg (modeling-library kernel) + fw_std_pkg, both from
// fw-hdl; fw_proto_wb_pkg (wb_proto_if, the bus boundary) from fw-proto-wb; and
// fw_proto_gpio_pkg (gpio_drive_if) from fw-proto-gpio.
`include "fw_hdl_macros.svh"
`include "wb_dma_spl_macros.svh"     // FW_MEM_IMP + FW_WB_PROTO_IMP providers

package wb_dma_spl_pkg;
    import fw_hdl_pkg::*;
    import fw_std_pkg::*;           // fw event/component kernel helpers
    export fw_std_pkg::*;
    import fw_proto_wb_pkg::*;      // wb_proto_if -- the engine's Wishbone bus boundary
    export fw_proto_wb_pkg::*;      // re-export so consumers see wb_proto_if too
    import fw_proto_gpio_pkg::*;    // gpio_drive_if (interrupt-level output API only;
    export fw_proto_gpio_pkg::*;    // the transactor bridges ride along unused -- the
                                    // GPIO xtor is instanced only at the wb_dma_spl edge)

    // ---- shared types (no dedicated types package) ---------------------------
    // Interrupt cause delivered through wb_dma_irq_if.
    typedef enum bit [1:0] { CAUSE_CHUNK, CAUSE_DONE, CAUSE_ERR } wb_dma_cause_e;

    // One interrupt event: which channel and why.
    typedef struct packed {
        bit [4:0]      channel;
        wb_dma_cause_e cause;
    } wb_dma_irq_evt_t;

    // ---- MMIO field layouts (2-state packed structs, MSB-first; spec Tables 6/7)
    // These are the typed faces of the channel registers, consumed by the fw-hdl
    // register model (`fw_reg #(T)`): a named field is just a slice of T, so the
    // host programs and the engine inspects bits by name (r.read().ch_en) with no
    // hand-maintained bit offsets. Bit positions match the OpenCores wb_dma RTL.
    typedef struct packed {
        bit [8:0] reserved;       // 31:23 RO
        bit       int_chk_done;   // 22 ROC  (chunk-done interrupt source)
        bit       int_done;       // 21 ROC  (transfer-done interrupt source)
        bit       int_err;        // 20 ROC  (error interrupt source)
        bit       ine_chk_done;   // 19 RW   (chunk-done interrupt enable)
        bit       ine_done;       // 18 RW
        bit       ine_err;        // 17 RW
        bit       rest_en;        // 16 RW
        bit [2:0] prio;           // 15:13 RW
        bit       err;            // 12 RO   (status: errored)
        bit       done;           // 11 RO   (status: complete)
        bit       busy;           // 10 RO   (status: transferring)
        bit       stop;           //  9 WO   (host abort pulse)
        bit       sz_wb;          //  8 RW
        bit       use_ed;         //  7 RW   (external descriptor)
        bit       ars;            //  6 RW   (auto-restart)
        bit       mode;           //  5 RW   (1 = HW handshake)
        bit       inc_src;        //  4 RW
        bit       inc_dst;        //  3 RW
        bit       src_sel;        //  2 RW   (0 = IF0, 1 = IF1)
        bit       dst_sel;        //  1 RW
        bit       ch_en;          //  0 RW   (channel enable)
    } wb_dma_csr_t;

    typedef struct packed {
        bit [6:0]  reserved1;     // 31:25 RO
        bit [8:0]  chk_sz;        // 24:16 RW  (chunk size in words; 0 => whole TOT_SZ)
        bit [3:0]  reserved0;     // 15:12 RO
        bit [11:0] tot_sz;        // 11:0  RW  (total transfer size in words)
    } wb_dma_sz_t;

    // Global CSR (register-file offset 0x00). Only PAUSE is implemented; the RTL
    // reads it back as {31'h0, paused}. Typed so callers say `.read().pause`.
    typedef struct packed {
        bit [30:0] reserved;      // 31:1 RO
        bit        pause;         //  0   RW  (pause all DMA transfers)
    } wb_dma_gcsr_t;

    // Interrupt mask / source vector (INT_MASK_A/B at 0x04/0x08, INT_SRC_A/B at
    // 0x0c/0x10): one bit per channel (bit N = channel N), MSB reserved. `.ch` is
    // the per-channel slice (was a bare [30:0] index).
    typedef struct packed {
        bit        reserved;      // 31   RO
        bit [30:0] ch;            // 30:0 per-channel interrupt bits
    } wb_dma_intvec_t;

    // ---- DMA-specific APIs (each ships a `FW_WB_DMA_*_IMP macro) -------------
    `include "wb_dma_hs_if.svh"
    `include "wb_dma_irq_if.svh"
    `include "wb_dma_int_if.svh"      // interrupt-change notification seam

    // ---- model components (one class per file) -------------------------------
    `include "wb_dma_ch.svh"          // per-channel register bank + working set
    `include "wb_dma_rf.svh"          // register file + host fw_mem_if provider
    `include "wb_dma_de.svh"          // data-mover engine (runnable; inlines arbitration)
    `include "wb_dma_irq.svh"         // interrupt propagation (de.irq -> irq_a/irq_b)
    `include "wb_dma.svh"             // top component (rf + de + irq)
endpackage

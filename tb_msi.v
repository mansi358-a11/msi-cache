// -----------------------------------------------------------------------------
// tb_msi.v : self-checking testbench for the dual-core MSI system
//
//  1. Directed tests for every MSI transition.
//  2. 3000 random operations with a golden memory model (one core at a time).
//  3. Concurrent stress test (both cores at once) - checked by the invariants.
//
//  Invariants checked EVERY cycle (see "invariant checker" below):
//    - if one cache holds a line in M, the other cache does not hold it at all
//    - a line in S has the same value as main memory
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_msi;
    localparam AW = 8, DW = 32, IDX = 3;
    localparam [1:0] I = 0, S = 1, M = 2;

    reg clk = 0, rst = 1;
    always #5 clk = ~clk;

    reg          req [0:1];
    reg          we  [0:1];
    reg [AW-1:0] addr[0:1];
    reg [DW-1:0] wd  [0:1];
    wire [1:0]   done;
    wire [DW-1:0] rd0, rd1;

    msi_system #(.AW(AW), .DW(DW), .IDX(IDX)) dut (
        .clk(clk), .rst(rst),
        .c0_req(req[0]), .c0_we(we[0]), .c0_addr(addr[0]), .c0_wdata(wd[0]),
        .c0_done(done[0]), .c0_rdata(rd0),
        .c1_req(req[1]), .c1_we(we[1]), .c1_addr(addr[1]), .c1_wdata(wd[1]),
        .c1_done(done[1]), .c1_rdata(rd1));

    integer errors = 0, checks = 0;

    // ------------------------------------------------------------------
    // helpers
    // ------------------------------------------------------------------
    // MSI state of `a` in cache `core` (I if the line holds another tag)
    function [1:0] state_of(input integer core, input [AW-1:0] a);
        reg [IDX-1:0] ix;
        begin
            ix = a[IDX-1:0];
            if (core == 0)
                state_of = (dut.u_c0.tag[ix] == a[AW-1:IDX]) ? dut.u_c0.st[ix] : I;
            else
                state_of = (dut.u_c1.tag[ix] == a[AW-1:IDX]) ? dut.u_c1.st[ix] : I;
        end
    endfunction

    function [DW-1:0] cached_data(input integer core, input [AW-1:0] a);
        begin
            cached_data = (core == 0) ? dut.u_c0.dat[a[IDX-1:0]]
                                      : dut.u_c1.dat[a[IDX-1:0]];
        end
    endfunction

    task check(input [255:0] name, input [DW-1:0] got, input [DW-1:0] exp);
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("FAIL  %0s : got %h expected %h  (t=%0t)", name, got, exp, $time);
            end
        end
    endtask

    // One CPU operation; blocks until the cache reports done.
    task automatic do_op(input integer core, input is_write,
                         input [AW-1:0] a, input [DW-1:0] data,
                         output [DW-1:0] rdata);
        begin
            @(negedge clk);
            req[core] = 1; we[core] = is_write; addr[core] = a; wd[core] = data;
            @(posedge clk);
            while (!done[core]) @(posedge clk);
            rdata = (core == 0) ? rd0 : rd1;      // sampled in the done cycle
            @(negedge clk);
            req[core] = 0; we[core] = 0;
        end
    endtask

    reg [DW-1:0] tmp;
    task rd(input integer core, input [AW-1:0] a, output [DW-1:0] r);
        do_op(core, 0, a, 0, r);
    endtask
    task wr(input integer core, input [AW-1:0] a, input [DW-1:0] d);
        do_op(core, 1, a, d, tmp);
    endtask

    // ------------------------------------------------------------------
    // invariant checker (runs every clock edge once reset is released)
    // ------------------------------------------------------------------
    integer ia;
    reg [1:0] s0, s1;
    always @(posedge clk) if (!rst) begin
        for (ia = 0; ia < (1<<AW); ia = ia + 1) begin
            s0 = state_of(0, ia[AW-1:0]);
            s1 = state_of(1, ia[AW-1:0]);
            if ((s0 == M && s1 != I) || (s1 == M && s0 != I)) begin
                errors = errors + 1;
                $display("FAIL  invariant: addr %0d has M in one cache and a copy in the other (s0=%0d s1=%0d) t=%0t", ia, s0, s1, $time);
            end
            if (s0 == S && cached_data(0, ia[AW-1:0]) !== dut.mem[ia]) begin
                errors = errors + 1;
                $display("FAIL  invariant: cache0 Shared copy of addr %0d differs from memory t=%0t", ia, $time);
            end
            if (s1 == S && cached_data(1, ia[AW-1:0]) !== dut.mem[ia]) begin
                errors = errors + 1;
                $display("FAIL  invariant: cache1 Shared copy of addr %0d differs from memory t=%0t", ia, $time);
            end
        end
    end

    // ------------------------------------------------------------------
    // tests
    // ------------------------------------------------------------------
    reg [DW-1:0] gold [0:(1<<AW)-1];
    reg [DW-1:0] r, r2;
    integer n, core, ad;
    localparam A = 8'd5;          // index 5, tag 0
    localparam B = 8'd13;         // index 5, tag 1  (conflicts with A)

    // random per-core driver for the concurrent test
    task automatic driver(input integer core, input integer count);
        integer j; reg [AW-1:0] a; reg [DW-1:0] dummy;
        begin
            for (j = 0; j < count; j = j + 1) begin
                a = $urandom % 16;                 // 16 hot addresses -> heavy sharing
                if ($urandom % 2) do_op(core, 1, a, {core[3:0], 4'h0, j[23:0]}, dummy);
                else              do_op(core, 0, a, 0, dummy);
            end
        end
    endtask

    initial begin
        $dumpfile("msi.vcd");
        $dumpvars(0, tb_msi);
        req[0] = 0; req[1] = 0; we[0] = 0; we[1] = 0;
        addr[0] = 0; addr[1] = 0; wd[0] = 0; wd[1] = 0;
        repeat (3) @(posedge clk);
        rst = 0;
        for (n = 0; n < (1<<AW); n = n + 1) gold[n] = 32'h1000_0000 + n;

        // --- T1: read miss -> Shared, data comes from memory ---
        rd(0, A, r);
        check("T1 read miss data", r, 32'h1000_0000 + A);
        check("T1 core0 state = S", state_of(0, A), S);
        check("T1 core1 state = I", state_of(1, A), I);

        // --- T2: second reader -> both Shared ---
        rd(1, A, r);
        check("T2 core1 read data", r, 32'h1000_0000 + A);
        check("T2 core0 state = S", state_of(0, A), S);
        check("T2 core1 state = S", state_of(1, A), S);

        // --- T3: write to a Shared line -> upgrade to M, other copy invalidated ---
        wr(0, A, 32'hAAAA_0001);
        check("T3 core0 state = M", state_of(0, A), M);
        check("T3 core1 state = I (invalidated)", state_of(1, A), I);
        check("T3 memory still old (write-back cache)", dut.mem[A], 32'h1000_0000 + A);

        // --- T4: read of a Modified line -> owner supplies data, M -> S ---
        rd(1, A, r);
        check("T4 core1 sees core0's write", r, 32'hAAAA_0001);
        check("T4 core0 downgraded to S", state_of(0, A), S);
        check("T4 core1 state = S", state_of(1, A), S);
        check("T4 memory updated by owner", dut.mem[A], 32'hAAAA_0001);

        // --- T5: write miss while the other cache holds M ---
        wr(0, A, 32'hAAAA_0002);                  // core0 S->M, core1 -> I
        wr(1, A, 32'hBBBB_0003);                  // core1 write miss, core0 owns M
        check("T5 core1 state = M", state_of(1, A), M);
        check("T5 core0 state = I", state_of(0, A), I);
        rd(0, A, r);
        check("T5 core0 reads core1's data", r, 32'hBBBB_0003);

        // --- T6: eviction of a Modified line -> write-back ---
        wr(0, A, 32'hCCCC_0004);                  // core0 holds A in M
        rd(0, B, r);                              // same index, evicts A
        check("T6 write-back reached memory", dut.mem[A], 32'hCCCC_0004);
        check("T6 B data correct", r, 32'h1000_0000 + B);
        check("T6 core0 now holds B in S", state_of(0, B), S);
        check("T6 core0 no longer holds A", state_of(0, A), I);
        rd(1, A, r);
        check("T6 core1 reads written-back value", r, 32'hCCCC_0004);

        // --- T7: ping-pong writes on one line ---
        for (n = 0; n < 20; n = n + 1) wr(n % 2, 8'd40, 32'hD000_0000 + n);
        rd(0, 8'd40, r); rd(1, 8'd40, r2);
        check("T7 core0 final value", r,  32'hD000_0013);
        check("T7 core1 final value", r2, 32'hD000_0013);

        // --- T8: random sequential ops vs golden model ---
        gold[A]    = 32'hCCCC_0004;      // value left by the directed tests
        gold[8'd40] = 32'hD000_0013;
        for (n = 0; n < 3000; n = n + 1) begin
            core = $urandom % 2;
            ad   = $urandom % 32;                 // 32 addresses, 4 tags x 8 indices
            if ($urandom % 2) begin
                wr(core, ad[AW-1:0], 32'hE000_0000 + n);
                gold[ad] = 32'hE000_0000 + n;
            end else begin
                rd(core, ad[AW-1:0], r);
                check("T8 random read vs golden", r, gold[ad]);
            end
        end

        // --- T9: both cores hammering shared addresses at the same time ---
        fork
            driver(0, 2000);
            driver(1, 2000);
        join
        // after the dust settles both cores must agree on every address
        for (n = 0; n < 16; n = n + 1) begin
            rd(0, n[AW-1:0], r); rd(1, n[AW-1:0], r2);
            check("T9 cores agree after stress", r, r2);
        end

        $display("--------------------------------------------");
        $display("checks run : %0d", checks);
        $display("errors     : %0d", errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $finish;
    end

    // watchdog
    initial begin
        #50_000_000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
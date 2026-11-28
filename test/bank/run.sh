#!/bin/sh
# Lock-step check of src/pg_bank.v's flop model against the IHP SRAM model.
# usage: PDK_ROOT=/path/to/pdk test/bank/run.sh
set -e
cd "$(dirname "$0")"
M=${PDK_ROOT:?set PDK_ROOT}/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/verilog
sed -e 's/module pg_bank (/module pg_bank_flop (/' -e 's/`ifdef PG_SRAM_MACRO/`ifdef PG_NEVER_DEFINED/' \
    ../../src/pg_bank.v > pg_bank_flop.v
iverilog -g2012 -DPG_SRAM_MACRO -DFUNCTIONAL -o tb tb_bank.v pg_bank_flop.v ../../src/pg_bank.v \
    "$M/RM_IHPSG13_1P_256x32_c2_bm_bist.v" "$M/RM_IHPSG13_1P_core_behavioral_bm_bist.v" 2>&1 | grep -v "Timing checks" || true
vvp -n tb | tee result.txt
grep -q " 0 mismatches" result.txt

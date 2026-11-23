#!/bin/sh
# Early area / netlist / timing estimate against the IHP CMOS5L library.
# usage: PDK_ROOT=/path/to/IHP-Open-PDK synth/run.sh
set -e
cd "$(dirname "$0")"
LIBDIR=${PDK_ROOT:?set PDK_ROOT}/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib
YOSYS=${YOSYS:-yosys}
sed "s#@LIB@#$LIBDIR/sg13cmos5l_stdcell_typ_1p20V_25C.lib#g" synth.ys.in > synth.ys
$YOSYS -q -l synth.log synth.ys
grep -E "Chip area|sequential" synth.log
for c in typ_1p20V_25C slow_1p08V_125C; do
  echo "timing estimate ($c):"
  python3 ../tools/gl/mini_sta.py gate_level_netlist.v "$LIBDIR/sg13cmos5l_stdcell_$c.lib"
done

#!/bin/sh
# Icarus Verilog 12 does not drive the delayed_* nets of $setuphold, which makes
# every IHP flop output X.  This makes a functional copy of the cell models with
# the delayed nets tied to the real pins (timing checks are irrelevant for a
# zero-delay functional gate-level simulation).
# usage: make_functional_models.sh $PDK_ROOT/ihp-sg13cmos5l out_dir
set -e
SRC=$1/libs.ref/sg13cmos5l_stdcell/verilog
mkdir -p "$2"
cp "$SRC/sg13cmos5l_udp.v" "$2/"
sed -E -e 's/^\s*wire delayed_[A-Za-z0-9_, ]*delayed_[A-Za-z0-9_]*;//' \
       -e '/\$setuphold|\$recrem|\$setup|\$hold|\$width|\$removal|\$recovery/d' \
       -e 's/delayed_([A-Za-z0-9_]+)/\1/g' \
       "$SRC/sg13cmos5l_stdcell.v" > "$2/sg13cmos5l_stdcell.v"

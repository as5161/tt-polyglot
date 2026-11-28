"""
LibreLane plugin for Polyglot's IHP SRAM instruction banks.

Adapted from kdp1965/ihp-um-janestreet-prism (librelane_plugin_prism_pdn.py,
odb_stripes.py, odb_sram_diodes.py) via josusanmartin/wirelingo
(librelane_plugin_protoemu.py), both Apache-2.0.  Thanks to Ken Pettit and
Josu San Martin, who worked out how to power and protect these macros on the
Tiny Tapeout SG13CMOS5L tile.  Changes here: only the three pieces Polyglot
needs are kept, and the scripts live in flow/.

* Project.ExtendPowerStripes: the tile PDN is Metal4 stripes only and pdngen
  carves its stripes around macros, so it cannot reach the SRAMs' Metal4
  power columns.  This step draws stripes over them (flow/odb_stripes.py).
* Project.DiodesOnSramPins: an antenna diode on each SRAM signal input (the
  SRAM LEF has no antenna data, so the rule-based repair cannot see those
  nets) (flow/odb_sram_diodes.py).
* A fix for netgen's LVS JSON escaping of the SRAM power pin names (VDD! ...).

LibreLane imports modules named librelane_plugin_* from the working directory
(the repository root, both locally and in the Tiny Tapeout GDS action).
src/config.json inserts the steps through meta.substituting_steps.
SPDX-License-Identifier: Apache-2.0
"""
import os
import json as _json
import re as _re
import types as _types
from decimal import Decimal
from typing import List, Optional

from librelane.config import Variable
from librelane.steps import Step
from librelane.steps.odb import OdbpyStep
from librelane.steps.openroad import DetailedPlacement
from librelane.steps.step import CompositeStep, StepError

HERE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "flow")


@Step.factory.register()
class ExtendPowerStripes(OdbpyStep):
    id = "Project.ExtendPowerStripes"
    name = "Extend Power Stripes Over SRAMs"

    def get_script_path(self):
        return os.path.join(HERE, "odb_stripes.py")

    config_vars = [
        Variable("EXTEND_STRIPES_LAYER", str, "The tile's vertical single-layer PDN layer (Metal4 on the CMOS5L tile).", default="Metal4"),
        Variable("EXTEND_STRIPES_SRAM_LAYER", Optional[str], "Layer of the IHP SRAM power columns when it differs from the stripe layer.", default=None),
        Variable("EXTEND_STRIPES_CLEARANCE", Decimal, "Spacing kept between a drawn stripe and the other net's macro rails or tile pins on the stripe layer.", units="µm", default=Decimal("0.24")),
        Variable("EXTEND_STRIPES_STACK_PITCH", Decimal, "Spacing of the via stacks along an SRAM power column.", units="µm", default=Decimal("10")),
        Variable("EXTEND_STRIPES_PIN_FACE_MARGIN", Decimal, "No rail via stack within this distance of a macro edge that carries pins; 0 leaves pdngen's stacks alone.", units="µm", default=Decimal("3")),
        Variable("EXTEND_STRIPES_SRAM_ALL_COLUMNS", bool, "Put a stripe on every legal supply column of an IHP SRAM.", default=False),
        Variable("EXTEND_STRIPES_SRAM_ARRAY_EVERY_OTHER", bool, "In each bit-cell array of an IHP SRAM, add a VPWR/VGND stripe pair on every other available pair position.", default=False),
    ]

    def get_command(self):
        cmd = super().get_command() + [
            "--layer", self.config["EXTEND_STRIPES_LAYER"],
            "--clearance", str(self.config["EXTEND_STRIPES_CLEARANCE"]),
            "--stack-pitch", str(self.config["EXTEND_STRIPES_STACK_PITCH"]),
            "--pin-face-margin", str(self.config["EXTEND_STRIPES_PIN_FACE_MARGIN"]),
        ]
        if self.config.get("EXTEND_STRIPES_SRAM_LAYER"):
            cmd += ["--sram-layer", self.config["EXTEND_STRIPES_SRAM_LAYER"]]
        cmd += ["--sram-all-columns" if self.config["EXTEND_STRIPES_SRAM_ALL_COLUMNS"] else "--sram-grid-columns"]
        if self.config["EXTEND_STRIPES_SRAM_ARRAY_EVERY_OTHER"]:
            cmd += ["--sram-array-every-other"]
        return cmd


@Step.factory.register()
class SramPinDiodePlacement(OdbpyStep):
    id = "Project.SramPinDiodePlacement"
    name = "Diodes on SRAM Input Pins"

    def get_script_path(self):
        return os.path.join(HERE, "odb_sram_diodes.py")

    def get_command(self) -> List[str]:
        cell, pin = self.config["DIODE_CELL"].split("/")
        return super().get_command() + ["--diode-cell", cell, "--diode-pin", pin, "--macro-prefix", "RM_IHPSG13"]


@Step.factory.register()
class DiodesOnSramPins(CompositeStep):
    id = "Project.DiodesOnSramPins"
    name = "Diodes on SRAM Pins"
    Steps = [SramPinDiodePlacement, DetailedPlacement]


@Step.factory.register()
class StopAfterGlobalRouting(Step):
    """Routing probe: replaces OpenROAD.DetailedRouting so a build ends about an
    hour in, right after global routing, with the congestion report printed.
    Gate for full builds: total overflow must be low enough for detailed
    routing to clear (Wirelingo's data: a few hundred at most).  Probe
    branches only; never on a branch meant to produce a GDS."""
    id = "Project.StopAfterGlobalRouting"
    name = "Routing probe: stop after global routing"
    inputs = []
    outputs = []

    def run(self, state_in, **kwargs):
        import glob
        run_dir = os.path.dirname(self.step_dir)
        logs = sorted(glob.glob(os.path.join(run_dir, "*-openroad-globalrouting", "*.log")))
        report = "(global routing log not found)"
        if logs:
            text = open(logs[-1]).read()
            i = text.find("Final congestion report")
            if i >= 0:
                report = text[i:i + 1400]
        m = state_in.metrics
        keys = [k for k in m if k.startswith("global_route__") or k.startswith("design__instance__utilization")
                or k in ("design__instance__count", "design__instance__count__class:buffer", "design__instance__count__class:antenna_cell")]
        summary = "\n".join(f"{k} = {m[k]}" for k in sorted(keys))
        msg = "ROUTING PROBE RESULT\n" + report + "\n" + summary
        print(msg)
        with open(os.path.join(self.step_dir, "probe.txt"), "w") as f:
            f.write(msg + "\n")
        raise StepError("routing probe finished (this branch stops after global routing on purpose)")


# --- netgen writes the IHP SRAM's power pin names (VDD!, VSS!, VDDARRAY!) into
# its LVS JSON with a stray backslash ("\VDD!"), which is not a valid JSON
# escape, and librelane.steps.netgen.LVS then dies in json.loads before the
# LVS checker runs.  Give that module a json namespace whose loads() doubles
# such backslashes first (the metrics only count entries; names do not matter).
import librelane.steps.netgen as _netgen  # noqa: E402

_BAD_ESCAPE = _re.compile(r'\\(\\|[^"\\/bfnrtu])')


def _repair_escapes(s):
    return _BAD_ESCAPE.sub(lambda m: '\\\\' if m.group(1) == '\\' else '\\\\' + m.group(1), s)


def _loads_repairing_escapes(s, *args, **kwargs):
    try:
        return _json.loads(s, *args, **kwargs)
    except _json.JSONDecodeError:
        return _json.loads(_repair_escapes(s), *args, **kwargs)


_netgen.json = _types.SimpleNamespace(
    loads=_loads_repairing_escapes, load=_json.load, dumps=_json.dumps, dump=_json.dump,
    JSONDecodeError=_json.JSONDecodeError,
)

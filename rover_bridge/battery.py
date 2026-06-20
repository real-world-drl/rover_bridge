# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Battery state-of-charge estimate for the rover's Li-ion pack.

The rover reports raw pack voltage + current over `tel/battery`; this turns
pack voltage into an approximate charge percentage. The default pack is 3S
(three Li-ion cells in series, e.g. 18650s): 12.6 V full, ~9.0 V empty.

Li-ion voltage is very flat through the mid-SoC range, so a linear V→% map is
wildly off in the middle. We interpolate a per-cell open-circuit-voltage (OCV)
table instead — a standard 18650 rest-voltage curve — and scale by cell count.

Caveat: OCV tables assume the cell is *at rest*. Under load the measured
voltage sags (IR drop), so the estimate reads low while the motors are pulling
current. Good enough for a "roughly how full" readout, not for fuel-gauge
accuracy. If you need better, IR-compensate using the reported current and a
measured pack resistance.
"""

from __future__ import annotations

# (cell_voltage, percent), descending by voltage. Typical 18650 OCV curve.
_OCV_TABLE = [
    (4.20, 100.0), (4.15, 95.0), (4.11, 90.0), (4.08, 85.0), (4.02, 80.0),
    (3.98, 75.0), (3.95, 70.0), (3.91, 65.0), (3.87, 60.0), (3.85, 55.0),
    (3.84, 50.0), (3.82, 45.0), (3.80, 40.0), (3.79, 35.0), (3.77, 30.0),
    (3.75, 25.0), (3.73, 20.0), (3.71, 15.0), (3.69, 10.0), (3.61, 5.0),
    (3.27, 0.0),
]


def percent_from_voltage(pack_voltage_v: float, cells: int = 3) -> float:
    """Estimate charge percentage (0–100) from total pack voltage.

    Args:
        pack_voltage_v: measured pack voltage across all cells in series.
        cells: number of series Li-ion cells (default 3 = 3S).
    """
    if cells <= 0:
        return 0.0
    cell_v = pack_voltage_v / cells

    if cell_v >= _OCV_TABLE[0][0]:
        return 100.0
    if cell_v <= _OCV_TABLE[-1][0]:
        return 0.0

    # Linear interpolation between the two bracketing table points.
    for (v_hi, p_hi), (v_lo, p_lo) in zip(_OCV_TABLE, _OCV_TABLE[1:]):
        if v_lo <= cell_v <= v_hi:
            frac = (cell_v - v_lo) / (v_hi - v_lo)
            return p_lo + frac * (p_hi - p_lo)
    return 0.0  # unreachable given the bounds checks above

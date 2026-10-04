"""Measurement with an honest interval.

Every number we emit is a `Measure`: point value + 95% interval + the error budget that
produced the interval, so a reader can see *why* the interval is as wide as it is.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np

Z95 = 1.96


@dataclass
class Measure:
    value: float
    sigma: float                  # 1-sigma total, same unit as value
    unit: str = "m"
    budget: dict = field(default_factory=dict)   # name -> 1-sigma contribution
    observed: bool = True
    note: str = ""

    @property
    def lo(self):
        return self.value - Z95 * self.sigma

    @property
    def hi(self):
        return self.value + Z95 * self.sigma

    def to_json(self, nd=3):
        r = lambda x: None if x is None or not np.isfinite(x) else round(float(x), nd)
        d = dict(value=r(self.value), ci95=[r(self.lo), r(self.hi)], sigma=r(self.sigma), unit=self.unit,
                 observed=self.observed)
        if self.budget:
            d["error_budget_1sigma"] = {k: r(v) for k, v in self.budget.items()}
        if self.note:
            d["note"] = self.note
        return d


def combine(value, unit="m", **terms) -> Measure:
    """Root-sum-square of independent 1-sigma terms."""
    terms = {k: float(v) for k, v in terms.items() if v is not None and np.isfinite(v)}
    s = float(np.sqrt(sum(v * v for v in terms.values())))
    return Measure(float(value), s, unit, terms)

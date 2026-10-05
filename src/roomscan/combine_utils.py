"""Small helpers on Measure objects."""
from __future__ import annotations

from .measure import combine


def add_term(m, note="", **terms):
    """Return a copy of Measure `m` with extra independent 1-sigma terms added to its budget."""
    out = combine(m.value, unit=m.unit, **{**m.budget, **terms})
    out.observed = m.observed
    out.note = "; ".join(x for x in (m.note, note) if x)
    return out

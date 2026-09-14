"""Adapted from EVA-CLIENT collection_alignment.py (Apache-2.0).

Modified for Nero: plain timestamp/value arrays, strict bounded coverage,
and causal hold for sent commands. See THIRD_PARTY_NOTICES.md.
"""

import bisect

import numpy as np


def nearest_index(times, timestamp):
    if not times:
        raise ValueError("empty sample stream")
    index = bisect.bisect_left(times, timestamp)
    if index == 0:
        return 0
    if index == len(times):
        return index - 1
    return index - 1 if timestamp - times[index - 1] <= times[index] - timestamp else index


def interpolate(times, values, timestamp, max_gap):
    index = bisect.bisect_left(times, timestamp)
    if index < len(times) and times[index] == timestamp:
        return np.asarray(values[index], dtype=np.float32).copy()
    if index == 0 or index == len(times):
        raise ValueError("feedback does not bracket sample time")
    duration = times[index] - times[index - 1]
    if not 0 < duration <= max_gap:
        raise ValueError("feedback interpolation gap too large")
    left, right = np.asarray(values[index - 1]), np.asarray(values[index])
    return (left + (right - left) * ((timestamp - times[index - 1]) / duration)).astype(np.float32)


def hold(times, values, timestamp, max_age=None):
    index = bisect.bisect_right(times, timestamp) - 1
    if index < 0 or (max_age is not None and timestamp - times[index] > max_age):
        raise ValueError("no fresh causal sample")
    return values[index]


def stream_stats(times):
    gaps = np.diff(np.asarray(times, dtype=np.float64))
    return {"samples": len(times), "estimated_hz": float(1 / np.mean(gaps)) if len(gaps) and np.mean(gaps) > 0 else 0.,
            "max_gap_s": float(np.max(gaps)) if len(gaps) else 0.,
            "p99_gap_s": float(np.quantile(gaps, .99)) if len(gaps) else 0.}

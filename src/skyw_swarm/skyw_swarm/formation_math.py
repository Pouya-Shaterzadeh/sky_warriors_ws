#!/usr/bin/env python3

import math

def get_offsets(formation_type, spacing, drone_count):
    """
    Returns a list of (dx, dy) offsets for the followers in the leader's local frame.
    X-axis is Forward, Y-axis is Left.
    The leader is always at (0,0).
    """
    offsets = [(0.0, 0.0)] # Leader (Drone 1)
    
    s = spacing
    form = formation_type.lower()

    if form == 'column':
        # Single file transit: (0,0), (-s, 0), (-2s, 0)
        for i in range(1, drone_count):
            offsets.append((-i * s, 0.0))

    elif form == 'line':
        # Side-by-side horizontal: (0,0), (0, -s), (0, s)
        side = -1
        idx = 1
        for i in range(1, drone_count):
            offsets.append((0.0, side * idx * s))
            side *= -1
            if side == -1:
                idx += 1

    elif form == 'triangle':
        # Equilateral Triangle (Tight spread: 60 deg total)
        # h = s * cos(30) = 0.866s
        # w = s * sin(30) = 0.5s
        h = s * 0.866
        offsets.append((-h, -0.5 * s)) 
        offsets.append((-h, 0.5 * s))  

    elif form == 'diamond':
        # Wide Diamond (Broad spread: 120 deg total)
        # h = s * cos(60) = 0.5s
        # w = s * sin(60) = 0.866s
        h = s * 0.5
        w = s * 0.866
        offsets.append((-h, -w)) 
        offsets.append((-h, w))  

    elif form == 'arrow_head':
        # Sharp Arrow (Narrow spread: 30 deg total)
        # h = s * cos(15) = 0.96s
        # w = s * sin(15) = 0.25s
        h = s * 0.96
        w = s * 0.25
        offsets.append((-h, -w))
        offsets.append((-h, w))

    else: # Default/V-Formation
        # Classic V (90 deg total)
        # h = s, w = s
        offsets.append((-s, -s)) 
        offsets.append((-s, s))  

    return offsets[:drone_count]

# Registry for naming consistency
FORMATION_BUILDERS = {
    "v": "Classic V (90 deg)",
    "line": "Horizontal Line (180 deg)",
    "triangle": "Equilateral Triangle (60 deg)",
    "diamond": "Wide Diamond (120 deg)",
    "arrow_head": "Sharp Arrow (30 deg)",
    "column": "Single File Transit",
}

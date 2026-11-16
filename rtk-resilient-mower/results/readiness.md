## RTK readiness survey

### As is: GO WITH MITIGATION (7.3% of the lawn at risk)

Based on 5 simulated missions over 356 m² of mowable lawn.

| Class | Area (m²) | Share of lawn | Meaning |
|---|---|---|---|
| Ready | 116 | 32.7% | RTK fixed at least 95% of the time: mowed at full speed |
| Degraded | 214 | 59.9% | Mowed, but slowly: RTK float or no fix for more than 5% of the time |
| At risk | 26 | 7.3% | Skipped and reported in most missions: needs a mitigation |
| Not visited | 0 | 0.1% | Never reached by the robot in these missions |

Mission time: 55 min (54–57), against 42 min with perfect positioning (+32%).
Grass neither mowed nor reported as deferred: 3.5% (edges the deck can't reach between passes).

| Problem zone | Area (m²) | Cause | What would fix it |
|---|---|---|---|
| 1. Along the two-storey walls | 10.2 | wall reflections: float fixes biased away from the wall | Keep the wider wall margin, or add a camera check for the strip along the wall. |
| 2. Under the maple dense core | 7.1 | dense canopy: no RTK solution at all | Add camera or LiDAR navigation here, a short boundary-wire island, prune the canopy, or leave it for hand mowing. |
| 3. Open lawn beside the maple canopy | 5.3 | open sky, but RTK still re-converging after a canopy or wall zone (4–10 s) | Plan lanes so the robot exits degraded zones into open lawn, not toward a hazard. |
| 4. Along the two-storey walls | 1.7 | wall reflections: float fixes biased away from the wall | Keep the wider wall margin, or add a camera check for the strip along the wall. |
| 5. Under the maple canopy | 0.9 | light canopy: mostly RTK float, brief fixed windows | The robot slows to 0.2 m/s and skips strips next to trunks; camera assist would recover them. |
| 6. Open lawn beside the two-storey walls | 0.8 | open sky, but RTK still re-converging after a canopy or wall zone (4–10 s) | Plan lanes so the robot exits degraded zones into open lawn, not toward a hazard. |

### Maple's dense core thinned: GO WITH MITIGATION (3.9% of the lawn at risk)

Based on 5 simulated missions over 356 m² of mowable lawn.

| Class | Area (m²) | Share of lawn | Meaning |
|---|---|---|---|
| Ready | 118 | 33.2% | RTK fixed at least 95% of the time: mowed at full speed |
| Degraded | 224 | 62.8% | Mowed, but slowly: RTK float or no fix for more than 5% of the time |
| At risk | 14 | 3.9% | Skipped and reported in most missions: needs a mitigation |
| Not visited | 0 | 0.1% | Never reached by the robot in these missions |

Mission time: 53 min (53–54), against 42 min with perfect positioning (+28%).
Grass neither mowed nor reported as deferred: 2.9% (edges the deck can't reach between passes).

| Problem zone | Area (m²) | Cause | What would fix it |
|---|---|---|---|
| 1. Along the two-storey walls | 10.2 | wall reflections: float fixes biased away from the wall | Keep the wider wall margin, or add a camera check for the strip along the wall. |
| 2. Along the two-storey walls | 1.7 | wall reflections: float fixes biased away from the wall | Keep the wider wall margin, or add a camera check for the strip along the wall. |
| 3. Under the maple canopy | 1.0 | light canopy: mostly RTK float, brief fixed windows | The robot slows to 0.2 m/s and skips strips next to trunks; camera assist would recover them. |
| 4. Open lawn beside the two-storey walls | 0.8 | open sky, but RTK still re-converging after a canopy or wall zone (4–10 s) | Plan lanes so the robot exits degraded zones into open lawn, not toward a hazard. |

Plus 0.1 m² of at-risk fragments under 0.5 m² each.

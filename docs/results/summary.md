### Task

| Arm controller | Trials finished | Grasped | Placed | Trials with every object placed | Placement error [mm] | Cycle per object [s] | Planning per object [s] |
|---|---|---|---|---|---|---|---|
| Joint trajectory (stock, position) | 10/10 | 30/30 | 30/30 | 10/10 | 0.7 (max 1.8) | 13.6 ± 2.1 | 0.24 |
| Computed torque (ours, effort) | 10/10 | 30/30 | 30/30 | 10/10 | 0.8 (max 1.6) | 13.8 ± 2.0 | 0.25 |

### Perception

| Shape | Objects | Classified correctly | Placed | Detection error [mm] |
|---|---|---|---|---|
| box | 40 | 40/40 | 40/40 | 0.7 (max 1.5) |
| cylinder | 20 | 20/20 | 20/20 | 1.0 (max 2.0) |

### Tracking (while the reference moves)

| Arm controller | Trials | Joint error RMS [mrad] | TCP error RMS [mm] | TCP error 99th percentile [mm] | TCP error at rest [mm] |
|---|---|---|---|---|---|
| Joint trajectory (stock, position) | 10 | 7.59 | 7.04 | 17.14 | 0.000 |
| Computed torque (ours, effort) | 10 | 1.02 | 0.53 | 1.62 | 0.016 |

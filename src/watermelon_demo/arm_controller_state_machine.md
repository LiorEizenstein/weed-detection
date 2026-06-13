# Arm Controller State Machine

## States

| State | Description |
|-------|-------------|
| `INIT` | Move arm to home position on startup |
| `SCAN_MOVE` | Move arm to next pre-defined scan pose |
| `WAITING_DETECTION` | Dwell at scan pose, wait for a weed detection |
| `MOVE_TO_WEED` | Correct arm position toward weed pixel centroid |
| `FIRE_LASER` | Publish `/laser_fire True`, trigger beam effect |
| `RETURN_HOME` | Move arm back to home position |

---

## Transitions

```
                        ┌─────────────────────────────────────────────┐
                        │                                             │
                        ▼                                             │
                    ┌────────┐                                        │
      startup ────► │  INIT  │                                        │
                    └────┬───┘                                        │
                         │ send arm to HOME_POSE                      │
                         ▼                                            │
                    ┌────────────┐                                    │
          ┌────────►│ SCAN_MOVE  │◄───────────────────────────────┐  │
          │         └─────┬──────┘                                │  │
          │               │ move to SCAN_POSES[i]                 │  │
          │               │ i = 0,1,2,3 (cycles)                  │  │
          │               ▼                                        │  │
          │     ┌──────────────────────┐                          │  │
          │     │  WAITING_DETECTION   │                          │  │
          │     │  (dwell scan_dwell   │                          │  │
          │     │   _time seconds)     │                          │  │
          │     └──────────┬───────────┘                          │  │
          │                │                                       │  │
          │     no weed    │  weed detected on /detections         │  │
          │   detected ────┤                                       │  │
          │   (timeout)    │                                       │  │
          │                ▼                                       │  │
          │      ┌──────────────────┐                             │  │
          │      │  MOVE_TO_WEED    │◄──────────────┐             │  │
          │      │                  │               │             │  │
          │      │  err_x,err_y     │  weed still   │             │  │
          │      │  too large       │  off-centre   │             │  │
          │      └────────┬─────────┘               │             │  │
          │               │                         │             │  │
          │    |err| < 5% │  correct joints,        │             │  │
          │    fire zone  │  re-check detection ────┘             │  │
          │               │                                       │  │
          │               ▼                                       │  │
          │      ┌──────────────────┐                             │  │
          │      │   FIRE_LASER     │                             │  │
          │      │                  │                             │  │
          │      │ publish          │                             │  │
          │      │ /laser_fire True │                             │  │
          │      └────────┬─────────┘                            │  │
          │               │ wait laser_fire_duration sec          │  │
          │               │ publish /laser_fire False             │  │
          │               ▼                                       │  │
          │      ┌──────────────────┐                             │  │
          │      │  RETURN_HOME     │                             │  │
          │      │                  │                             │  │
          │      │ move to          ├─────────────────────────────┘  │
          │      │ HOME_POSE        │  done → next scan pose         │
          │      └──────────────────┘                                │
          │                                                          │
          └──────────────────────────────────────────────────────────┘
                    all scan poses visited → loop back
```

---

## Key Parameters (from `demo_params.yaml`)

| Parameter | Default | Effect |
|-----------|---------|--------|
| `scan_dwell_time` | `2.0 s` | How long to wait for a detection at each scan pose |
| `laser_fire_duration` | `1.0 s` | How long the laser stays on before moving home |

## Key Constants (from `arm_controller_node.py`)

| Constant | Value | Effect |
|----------|-------|--------|
| `SCAN_POSES` | 4 poses | Joint angles (rad) covering the four field sectors |
| `HOME_POSE` | `[0, -1.57, 0, -1.57, 0, 0]` | Safe neutral position |
| `PAN_GAIN` | `0.3 rad` | Correction per normalised pixel error in X |
| `LIFT_GAIN` | `0.2 rad` | Correction per normalised pixel error in Y |
| `FIRE_ZONE_FRAC` | `0.05` | Weed must be within 5% of image centre to fire |

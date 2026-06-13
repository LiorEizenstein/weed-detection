# Weed-Centering Control Loop — State Machine

State machine for error correction once the camera identifies a weed. The arm iterates a pixel-error correction loop, adjusting one joint axis per tick, until the weed is centred under the camera, then fires the laser.

```mermaid
stateDiagram-v2
    direction TB

    [*] --> WAITING_DETECTION : arm reaches scan pose

    WAITING_DETECTION --> MOVE_TO_WEED : weed detected\n(cx, cy from /detections)
    WAITING_DETECTION --> SCAN_MOVE : timeout (scan_dwell_time)\nno weed seen

    state MOVE_TO_WEED {
        direction TB
        [*] --> ComputeError
        ComputeError : Compute normalised pixel error\nerr_x = (cx − W/2) / (W/2)\nerr_y = (cy − H/2) / (H/2)

        ComputeError --> CentreCheck
        CentreCheck : |err_x| < 0.20\nAND |err_y| < 0.20 ?

        CentreCheck --> DominantAxis : NO — weed off-centre
        CentreCheck --> [*] : YES → FIRE_LASER

        DominantAxis : Which axis dominates?\n|err_x| ≥ |err_y| ?

        DominantAxis --> CorrectX : YES\npan[0] −= err_x × PAN_GAIN (0.10 rad)
        DominantAxis --> CorrectY : NO\nwrist_2[4] −= err_y × WRIST2_GAIN (0.10 rad)

        CorrectX --> ClampJoints
        CorrectY --> ClampJoints
        ClampJoints : Clamp joints\npan ∈ [−π, π]\nwrist_2 ∈ [−2.5, −0.5]

        ClampJoints --> SendTrajectory
        SendTrajectory : /follow_joint_trajectory\n(WEED_MOVE_TIME = 1.0 s)

        SendTrajectory --> WaitAction
        WaitAction : Wait for action result\n(busy = True)

        WaitAction --> ComputeError : action done,\ndetection still present

        WaitAction --> LostWeed : detection dropped
        LostWeed : Detection absent\ncheck timeout

        LostWeed --> WaitAction : < 8 s → wait for\nnext camera frame
        LostWeed --> [*] : ≥ 8 s → SCAN_MOVE\n(lost weed)
    }

    MOVE_TO_WEED --> FIRE_LASER : centred ✓
    MOVE_TO_WEED --> SCAN_MOVE : weed lost (8 s timeout)

    FIRE_LASER : Publish /laser_fire = True\nrecord fire_start time

    FIRE_LASER --> FIRING

    state FIRING {
        direction LR
        [*] --> BeamOn
        BeamOn : Laser beam on\nelapsed < laser_fire_duration (1.0 s)
        BeamOn --> BeamOn : still within duration
        BeamOn --> Done : elapsed ≥ laser_fire_duration
        Done : Publish /laser_fire = False\nBlacklist current pan angle\n(TREATED_ZONE_RAD = 0.25 rad)
    }

    FIRING --> SCAN_MOVE : weed treated ✓

    SCAN_MOVE : Advance to next scan pose\n(skip if near a treated pan angle)
    SCAN_MOVE --> WAITING_DETECTION : arm reaches next pose
```

## Control loop parameters

| Parameter | Value | Role |
|---|---|---|
| `FIRE_ZONE_FRAC` | 0.20 | Dead-zone threshold — fires when weed is within 20% of image centre |
| `PAN_GAIN` | 0.10 rad/px | Correction step size for X-axis (`shoulder_pan`) |
| `WRIST2_GAIN` | 0.10 rad/px | Correction step size for Y-axis (`wrist_2`) |
| `WEED_MOVE_TIME` | 1.0 s | Duration of each correction move |
| `WEED_LOCK_TIMEOUT` | 8.0 s | Give up on lost weed after this long |
| `TREATED_ZONE_RAD` | 0.25 rad | Blacklist radius around a fired pan angle |

## Why single-axis correction per tick

`shoulder_pan` rotation shifts the image in both X and Y, so correcting both axes simultaneously creates a coupled feedback loop that can diverge. Correcting only the dominant error axis per step decouples the loop and ensures convergence.

## Source

Implemented in [`src/watermelon_demo/watermelon_demo/arm_controller_node.py`](../src/watermelon_demo/watermelon_demo/arm_controller_node.py), `State.MOVE_TO_WEED` branch of `_tick()`.

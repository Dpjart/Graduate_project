# ROV thruster-layout comparison

Layouts: 4, 5, 6_45. Same hull, MPC weights and thruster rating for every layout.
Duration 35 s per experiment. Current 0.20 m/s (NED [0 0.2 0]) on [8, 14) s in experiment 3. Experiment 4 starts offset by [0.6 -0.5 0.3] m and 0.50 rad yaw. Experiment 5: thruster f1 fails at 10 s.

Errors are reference minus true state. RMSE includes the start-up transient. Convergence = time after which the 3D error stays within 0.05 m until the end (NaN = ends outside). Recovery = time after the current stops until the error stays within 0.05 m for 1.0 s.

## Experiment 1: Straight line

| Layout | 3D RMSE (m) | Max (m) | Final (m) | Convergence (s) | Recovery (s) | After-failure RMSE (m) | Max pitch (deg) | Saturation |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4 | 0.0021 | 0.0253 | 0.0000 | 0.00 | NaN | NaN | 0.41 | 0% |
| 5 | 0.0021 | 0.0253 | 0.0000 | 0.00 | NaN | NaN | 0.41 | 0% |
| 6_45 | 0.0021 | 0.0253 | 0.0000 | 0.00 | NaN | NaN | 0.41 | 0% |

## Experiment 2: Helix

| Layout | 3D RMSE (m) | Max (m) | Final (m) | Convergence (s) | Recovery (s) | After-failure RMSE (m) | Max pitch (deg) | Saturation |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4 | 0.1488 | 0.1674 | 0.1674 | NaN | NaN | NaN | 0.37 | 0% |
| 5 | 0.0019 | 0.0227 | 0.0002 | 0.00 | NaN | NaN | 0.37 | 0% |
| 6_45 | 0.0019 | 0.0227 | 0.0002 | 0.00 | NaN | NaN | 0.37 | 0% |

## Experiment 3: Helix with current

| Layout | 3D RMSE (m) | Max (m) | Final (m) | Convergence (s) | Recovery (s) | After-failure RMSE (m) | Max pitch (deg) | Saturation |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4 | 0.1998 | 0.5067 | 0.1669 | NaN | NaN | NaN | 1.04 | 0% |
| 5 | 0.0023 | 0.0227 | 0.0002 | 0.00 | 0.00 | NaN | 0.37 | 0% |
| 6_45 | 0.0023 | 0.0227 | 0.0002 | 0.00 | 0.00 | NaN | 0.37 | 0% |

## Experiment 4: Initial offset

| Layout | 3D RMSE (m) | Max (m) | Final (m) | Convergence (s) | Recovery (s) | After-failure RMSE (m) | Max pitch (deg) | Saturation |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4 | 0.1757 | 0.8367 | 0.1673 | NaN | NaN | NaN | 17.28 | 0% |
| 5 | 0.0873 | 0.8367 | 0.0002 | 1.42 | NaN | NaN | 14.82 | 1% |
| 6_45 | 0.0826 | 0.8367 | 0.0002 | 1.37 | NaN | NaN | 10.05 | 1% |

## Experiment 5: Thruster failure

| Layout | 3D RMSE (m) | Max (m) | Final (m) | Convergence (s) | Recovery (s) | After-failure RMSE (m) | Max pitch (deg) | Saturation |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4 | 0.4853 | 0.6767 | 0.5397 | NaN | NaN | 0.5707 | 40.06 | 0% |
| 5 | 0.2159 | 0.4281 | 0.4071 | NaN | NaN | 0.2554 | 64.38 | 8% |
| 6_45 | 0.0019 | 0.0227 | 0.0002 | 0.00 | NaN | 0.0002 | 0.37 | 0% |

NaN = not applicable or not reached. Full signals: <layout>/experiment_k.mat.

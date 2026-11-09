# Results: `main` (20 paired seeds x 3 shields)

ERROR trials: 0

| Shield | Collisions (95% Wilson CI) | Reached goal (95% CI) | Timeouts | Median time to goal (s) | Median min clearance (m) | Mean interventions |
|---|---|---|---|---|---|---|
| No shield | 13/20 (0.43–0.82) | 7/20 (0.18–0.57) | 0/20 | 17.7 | 0.007 | 0.0 |
| Brake shield | 0/20 (0.00–0.16) | 7/20 (0.18–0.57) | 13/20 | 17.7 | 0.202 | 1.6 |
| CBF-QP shield | 0/20 (0.00–0.16) | 12/20 (0.39–0.78) | 8/20 | 18.1 | 0.142 | 1.6 |

## Paired tests (same seed = same world)

| Comparison | Only first | Only second | p (exact) |
|---|---|---|---|
| collision off vs brake | 13 | 0 | 0.0002 |
| collision off vs cbf | 13 | 0 | 0.0002 |
| goal cbf vs brake | 5 | 0 | 0.0625 |
| goal cbf vs off | 5 | 0 | 0.0625 |

min_clearance cbf vs brake (Wilcoxon): median difference -0.059, p = 0.0936

time_to_goal cbf vs off, seeds where both reach (Wilcoxon): median difference +0.136, p = 0.0625, n = 7

## Outcomes split by whether a seeded obstacle blocks the straight path

| Shield | Path | Goal | Timeout | Collision |
|---|---|---|---|---|
| Brake shield | clear | 6 | 0 | 0 |
| Brake shield | blocked | 1 | 13 | 0 |
| CBF-QP shield | clear | 6 | 0 | 0 |
| CBF-QP shield | blocked | 6 | 8 | 0 |
| No shield | clear | 6 | 0 | 0 |
| No shield | blocked | 1 | 0 | 13 |

## Per-seed outcomes

| Seed | blockers | min abs lateral (m) | off | brake | cbf |
|---|---|---|---|---|---|
| 0 | 0 | nan | GOAL | GOAL | GOAL |
| 1 | 1 | 0.101 | COLLISION | TIMEOUT | TIMEOUT |
| 2 | 1 | 0.340 | COLLISION | TIMEOUT | GOAL |
| 3 | 1 | 0.088 | COLLISION | TIMEOUT | TIMEOUT |
| 4 | 0 | nan | GOAL | GOAL | GOAL |
| 5 | 1 | 0.484 | GOAL | GOAL | GOAL |
| 6 | 1 | 0.059 | COLLISION | TIMEOUT | TIMEOUT |
| 7 | 1 | 0.269 | COLLISION | TIMEOUT | GOAL |
| 8 | 0 | nan | GOAL | GOAL | GOAL |
| 9 | 1 | 0.253 | COLLISION | TIMEOUT | GOAL |
| 10 | 1 | 0.142 | COLLISION | TIMEOUT | TIMEOUT |
| 11 | 2 | 0.119 | COLLISION | TIMEOUT | TIMEOUT |
| 12 | 2 | 0.251 | COLLISION | TIMEOUT | TIMEOUT |
| 13 | 1 | 0.370 | COLLISION | TIMEOUT | GOAL |
| 14 | 2 | 0.405 | COLLISION | TIMEOUT | GOAL |
| 15 | 0 | nan | GOAL | GOAL | GOAL |
| 16 | 1 | 0.039 | COLLISION | TIMEOUT | TIMEOUT |
| 17 | 0 | nan | GOAL | GOAL | GOAL |
| 18 | 2 | 0.041 | COLLISION | TIMEOUT | TIMEOUT |
| 19 | 0 | nan | GOAL | GOAL | GOAL |

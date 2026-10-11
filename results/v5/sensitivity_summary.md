Sensitivity, adaptive (tight budget), office_loop_01, fog_severe; static fusion reference: ATE 0.0746 m, compute energy 4838 J.

| parameter | delta | runs | successful | ate_m | tracked | compute_energy_j | d_ate_pct | d_tracked_points | d_energy_pct | energy_below_static_fusion | ate_below_static_fusion |
|---|---|---|---|---|---|---|---|---|---|---|---|
| (frozen, main batch) | 0.0 | 3 | 3 | 0.0975 | 0.9997 | 3934.8387 | 0.0 | 0.0 | 0.0 | True | False |
| alpha | -0.2 | 2 | 2 | 0.0706 | 0.9995 | 3939.3326 | -27.5461 | -0.0229 | 0.1142 | True | True |
| alpha | 0.2 | 2 | 1 | 0.0338 | 0.9314 | 3948.6558 | -65.3575 | -6.8308 | 0.3511 | True | True |
| beta | -0.2 | 2 | 2 | 0.0552 | 0.9994 | 3931.9359 | -43.3886 | -0.0324 | -0.0738 | True | True |
| beta | 0.2 | 2 | 1 | 0.0551 | 0.9302 | 3950.6227 | -43.4562 | -6.9539 | 0.4011 | True | True |
| gamma | -0.2 | 2 | 1 | 0.0555 | 0.9369 | 3946.4662 | -43.059 | -6.2857 | 0.2955 | True | True |
| gamma | 0.2 | 2 | 1 | 0.0431 | 0.9318 | 3960.3015 | -55.7466 | -6.797 | 0.6471 | True | True |
| tau_Q | -0.2 | 2 | 2 | 0.0805 | 0.9998 | 3936.4063 | -17.4144 | 0.0065 | 0.0398 | True | False |
| tau_Q | 0.2 | 2 | 1 | 0.095 | 0.934 | 3963.5606 | -2.5386 | -6.5697 | 0.7299 | True | False |
| tau_E1 | -0.2 | 2 | 0 |  | 0.8534 | 4191.5055 |  | -14.6344 | 6.5229 | True |  |
| tau_E1 | 0.2 | 2 | 0 |  | 0.5234 | 3906.8495 |  | -47.6323 | -0.7113 | True |  |
| tau_E2 | -0.2 | 2 | 2 | 0.0487 | 0.9998 | 4034.1862 | -50.068 | 0.0073 | 2.5248 | True | True |
| tau_E2 | 0.2 | 2 | 0 |  | 0.8532 | 3910.7934 |  | -14.6521 | -0.6111 | True |  |
| thermal_priority_weight | -0.2 | 2 | 1 | 0.1062 | 0.9325 | 3953.6172 | 8.9491 | -6.7235 | 0.4772 | True | False |
| thermal_priority_weight | 0.2 | 2 | 1 | 0.0305 | 0.9296 | 3938.7355 | -68.7469 | -7.0154 | 0.099 | True | True |

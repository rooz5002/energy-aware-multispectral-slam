| parameter | value | note |
|---|---|---|
| alpha, beta, gamma (Eq. 1) | 0.4, 0.4, 0.2 | fixed |
| tau_Q (normalised quality) | 0.6 | tuned (stage 1) |
| tau_E1, tau_E2 (fraction of E0) | 0.5, 0.25 | tuned (stage 2) |
| hysteresis band | 0.02 | fixed |
| minimum dwell (s) | 2.0 | fixed |
| thermal-priority weight | 0.7 | fixed |
| minimum features per modality | 50 | fixed |
| thermal-off rule | True | tuned (stage 2) |
| recovery hold (s) | 2.0 | fixed |
| Level 0 (normal) | 1200 features; min. keyframe interval 0 frames; local BA every 1 keyframe(s); loop closure full; every 1 frame(s) processed | Level 0 fixed, Levels 1-2 tuned (stage 2) |
| Level 1 (energy_saving) | 800 features; min. keyframe interval 30 frames; local BA every 1 keyframe(s); loop closure reduced; every 1 frame(s) processed | Level 0 fixed, Levels 1-2 tuned (stage 2) |
| Level 2 (tracking_priority) | 700 features; min. keyframe interval 60 frames; local BA every 1 keyframe(s); loop closure off; every 2 frame(s) processed | Level 0 fixed, Levels 1-2 tuned (stage 2) |
| RGB-D sensor power (W) | 2.0 | D435, always on |
| thermal sensor power (W) | 0.6 | Boson 320-class, times u_TH |
| E0, tight budget | 1.00 x median measured energy of static fusion on the same sequence and profile | rule |
| E0, generous budget | 2.1 x the same median (= 1.05 / (1 - tau_E1)) | rule |
| tuned on | office_loop_tune | setting q0.6_e0.5-0.25_f800-700_k30-60_ba1_off1_s2, constraints met: False |

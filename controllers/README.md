# Controller submissions

| Method | Unscaled | Scaled 2.8 |
|---|---|---|
| PID | [file](unscaled/student_controller2.py) | [file](scaled_2p8/student_controller2.py) |
| LQI | [file](lqi/unscaled/student_controller2.py) | [file](lqi/scaled_2p8/student_controller2.py) |
| MPC | [file](mpc/unscaled/student_controller2.py) | [file](mpc/scaled_2p8/student_controller2.py) |

Copy one file to the TA setup as `student_controller2.py`. Restart the application
when changing controllers so Python does not reuse an earlier imported module.
The supplied environment files remain unchanged.

See [analysis and testing guidance](../CONTROLLER_COMPARISON.md), especially the
MPC dependency requirement and distinction between optimizer and fallback runs.
Groupmate files were relocated unchanged; original filenames/hashes are in
`GROUPMATE_IMPORTS.json`. `src/build_variants.py` builds the PID pair only.

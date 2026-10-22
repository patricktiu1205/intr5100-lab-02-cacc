# Controller candidates

| Method | Scale 1 | Scale 2.8 |
| --- | --- | --- |
| PID | [candidate](pid/scale_1/student_controller2.py) | [candidate](pid/scale_2p8/student_controller2.py) |
| LQI | [candidate](lqi/scale_1/student_controller2.py) | [candidate](lqi/scale_2p8/student_controller2.py) |
| MPC | [candidate](mpc/scale_1/student_controller2.py) | [candidate](mpc/scale_2p8/student_controller2.py) |

All six algorithms and their original file bytes are retained. Folder names identify the method and scale hypothesis; neither hypothesis is a measured platform constant. Copy exactly one candidate beside the TA entry point as `student_controller2.py`, then restart the application to avoid reusing an imported module.

PID and LQI use the supplied route helper. MPC additionally requires NumPy, CVXPY and OSQP for its optimizer; internal feedback fallback must be identified separately when interpreting results. The physical logs do not record its internal solver mode.

[Import provenance](import_manifest.json) retains groupmate file hashes and original paths. [Design comparison](../docs/controller_comparison.md) describes the methods. `src/build_pid_variants.py` rebuilds only the PID pair from the maintained source; it does not modify LQI or MPC.

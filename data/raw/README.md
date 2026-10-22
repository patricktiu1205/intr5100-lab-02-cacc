# Preserved physical-run data

The three uploaded ZIPs were expanded here and removed after every data file
was verified byte-for-byte using SHA-256. Each folder's `archive_manifest.json`
records the original archive name/hash and every retained file/hash. macOS
resource-fork sidecars were omitted; all eight run data files were preserved.

- `group1_deployment_error_20261020_135916`: earlier run with Group 1's controller
  loaded by mistake; diagnostic evidence only, not a Group 4 PID result.
- `group4_pid_2p8`: corrected Group 4 PID 2.8 run, originally `group4.zip`.
- `group4_mpc_2p8`: run identified by the user as MPC 2.8, originally
  `experiment2_20261020_145126.zip`.
- `group4_lqi_2p8`: run identified by the user as LQI 2.8, originally
  `experiment2_20261020_150346.zip`; eight run files verified before removing the ZIP.

Analysis scripts read these folders directly; ZIPs are no longer required.

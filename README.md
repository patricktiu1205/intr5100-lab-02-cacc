# INTR 5100 Lab 02 CACC

Look-ahead car-following control on the SC2I testbed, comparing predictive PID, LQI and MPC. This repository contains the controller candidates, recorded experiments, saved analyses and revised report.

## Report

- [Report PDF](report/lab_02_report.pdf) and [editable Word version](report/lab_02_report.docx): seven pages, styled consistently with Lab I.
- [LaTeX source](report/source/main.tex), [report builder](src/build_report.py) and [review notes](review/review_notes.md).
- [Lab instructions](docs/lab_02_guidelines.pdf).

The report uses one Group 4 physical run per controller. Target gaps, initial conditions and native safety thresholds differ. It separates those observations from historical simulations and does not claim a universal controller winner. Physical records, historical results and controller algorithms have not been retuned during organization or report revision.

## Repository layout

| Folder | Contents |
| --- | --- |
| `controllers/` | Six deployment candidates, grouped by method and scale hypothesis |
| `src/` | Maintained PID source, simulation/analysis tools, report builder and checks |
| `environment/` | Supplied Lab II support code, GUI entry point and a local route map |
| `data/raw/` | Three Group 4 recordings plus the excluded deployment-error recording, with source manifests |
| `results/` | Recorded-run analyses, aligned event CSVs and saved recorded-input simulations |
| `simulation_results/` | Historical standalone and six-controller simulation outputs |
| `diagnostics/` | Earlier deployment-error diagnostics |
| `report/` | Current PDF/DOCX, source/figures and archived report/template evidence |
| `docs/` | Controller design, lab instructions and analysis summaries |
| `references/` | Research papers and reading guide |
| `review/` | Review findings, organization records and verification evidence |

Folders and descriptive filenames use lowercase snake_case. `README.md`, `AGENTS.md`, dependency manifests and the TA-required `student_controller2.py` retain their conventional names. The `scale_1` and `scale_2p8` labels describe telemetry/command hypotheses, not established physical conversion factors.

## Shared course environment

Use the single Conda environment for all INTR 5100 work:

```sh
conda activate intr5100
```

For a standalone clone, create that same environment if needed:

```sh
conda env create -f environment.yml
conda activate intr5100
```

`environment.yml` is a snapshot of the course-root dependency specification. In the full course folder, maintain that root specification and sync this copy before publishing. The older `requirements*.txt` files record earlier setups and optional dependency groups.

## Build the report

From the repository root:

```sh
PYTHONDONTWRITEBYTECODE=1 python src/build_report.py
```

The builder reads saved records, verifies their hashes and metrics, produces DOCX/LaTeX/figures, and compiles the PDF using Tectonic. It does not optimize controllers, refit the response model or rerun experimental simulations. Temporary compiler and plotting-cache files are removed automatically. Use `--offline` when the TeX resources are cached, or `--skip-pdf` to build only the editable sources.

## Verify code and deployment compatibility

```sh
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s src -p test_pid_controller.py -v
PYTHONDONTWRITEBYTECODE=1 python src/verify_controller_integration.py --all
```

The six unit checks cover the PID/TA IDM replica, state/history handling, route logic, stale telemetry, standalone imports and a synthetic closed-loop check. The GUI compatibility check uses copied support files, offscreen Qt, synthetic telemetry and in-memory command sinks. It opens no vehicle/network clients, removes its temporary copies and writes diagnostics to `review/dropin_checks.json`, leaving historical scientific outputs unchanged.

To submit a candidate, choose its file in [the controller index](controllers/README.md), copy it beside the TA entry point as `student_controller2.py`, and restart the application. MPC needs NumPy, CVXPY and OSQP for optimization; its internal fallback is a separate operating mode.

## Historical analysis

Start with [the three-controller analysis](docs/three_controller_analysis.md), [PID/MPC comparison](docs/pid_mpc_comparison.md), [six-controller simulations](docs/six_controller_simulation_results.md) and [PID design](docs/pid_controller_design.md). These document completed analyses. Running their simulation scripts creates new numerical outputs; it is unnecessary for rebuilding the current report.

The original Git history is preserved. The repository was renamed from `intr5100-lab2-pid` to `intr5100-lab-02-cacc` to reflect its full scope. Research PDFs and supplied course files retain their original attribution; no new redistribution license is assigned to them.

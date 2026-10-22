# Lab II report review

Reviewed and revised October 22, 2026 using the shared `intr5100` Conda environment. Scope is the Lab II guidelines, original draft, controller files, three Group 4 raw-run archives and saved physical/simulation analyses. The experimental results are fixed. No fitting, controller tests or simulations were run.

## Findings

The draft's headline physical numbers are consistent with the stored evidence. The report needed a shorter structure and clearer separation between measured trajectories, assumed controller policies and historical simulations. It also needed layout alignment with Lab I. The previous Word draft renders to 19 pages in the current renderer; both revised versions fit seven pages.

- Verified every file in the three raw-run archive manifests, active decision counts and minimum/median/maximum gaps against original commands.
- Verified the four event sample counts, own-target RMSEs and speed-variation ratios against retained event CSVs, and saved held-out displacement errors against the stored model-validation CSVs.
- Retained the different spacing policies: PID/LQI target `300 + 1.8v`, MPC `250 + 1.2v`, under the 2.8 speed hypothesis. Smaller gaps and own-target error do not provide a common-policy ranking.
- Retained the native safety thresholds, PID 80 cu and MPC/LQI 50 cu. All active observed gaps exceed a common 80-cu reference; this is an observation from these runs, not a safety guarantee.
- Physical logs identify callback operation but do not record MPC optimizer/fallback status or deployed source hashes. The revised report consistently labels the physical run accordingly. Saved MPC simulations also include protective-stop decisions.
- Removed the draft's assertion that a standard-library-only submission rule applies. The supplied guideline specifies the unchanged callback interface and supporting files, and permits MPC; it states no such library restriction. The dependency and fallback facts remain in the controller discussion.
- Separated the 27 previously saved simulation cases and source-specific response-model fits from physical measurements. Their results were reused, not regenerated. Within-run validation, unequal targets, interpolation and model mismatch limit the conclusions.
- Condensed repeated caveats and prospective testing discussion. Kept minimum gaps, event tracking, three-vehicle speed propagation and local flow proxies. Command increments remain a demand diagnostic; acceleration/jerk, fuel savings, network capacity and general string stability are not established by these records.

## Revised files

The current reports are `../report/lab_02_report.pdf` and `../report/lab_02_report.docx`. `../report/source/main.tex` and `../src/build_report.py` provide reproducible sources. One content model generates both formats. The PDF follows Lab I's A4, 20-mm margins, 11-point serif text, black headings and clean table/figure style. The Word version uses the same content, page geometry and restrained typography with editable tables and equations.

The original draft is unchanged. `Report_Revision_Checks.json` records hashes for 157 protected data, controller, environment, analysis-code and result files, all unchanged. Report tables, the descriptive plot, both document formats and all final pages were checked. Temporary build and rendering files were removed after verification.

## Subsequent repository organization

The later Git publication task renamed and grouped report/controller files, made the map and GUI support self-contained, and updated path handling and report compilation. The 157-file snapshot above records the earlier report review before that refactor. Its original paths can be resolved using `organization_manifest.json`. Current verification is in `publication_checks.json` and `report_build_checks.json`; physical data, historical result bytes, controller algorithms and the original draft remain unchanged.

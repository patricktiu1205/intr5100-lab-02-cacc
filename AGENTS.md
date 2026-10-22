# Lab II workspace instructions

- Use the shared Conda environment `intr5100` for all Python work. Do not create a per-lab environment.
- Preserve raw data, source manifests, historical numerical results and the six controller candidate files. Recorded experiments are fixed; do not change their outcomes or label newly generated results as original tests.
- `src/pid_controller.py` is the maintained PID source. Regenerate deployment variants only for an explicitly requested controller change. Keep the required `student_controller2.py` deployment filename.
- Use lowercase snake_case for descriptive filenames. Put final reports in `report/`, analysis/design notes in `docs/`, verification evidence in `review/` and report source figures in `report/source/figures/`.
- Align report styling with Lab I: A4, 20-mm margins, readable 11-point serif text, black headings and clean tables/figures. Keep content concise and separate physical observations from saved model results.
- Keep a standalone clone functional: no links to files outside the repository and no dependencies on a sibling course folder.
- In the full course workspace the course-root `environment.yml` is authoritative. Sync its repository copy before publishing.
- Remove temporary build files, page previews, test sandboxes, caches and uploaded ZIPs after their contents have been verified. Keep diagnostic evidence, scientific data/results, source files and final deliverables.

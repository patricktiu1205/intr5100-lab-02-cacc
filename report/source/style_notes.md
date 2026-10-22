# Report style and provenance

The report follows the user's request to align Lab II with Lab I: A4 portrait, 20-mm margins, 11-point serif body, black headings, a centered title block and restrained figures. PDF uses Latin Modern/booktabs through LaTeX; editable Word uses Times New Roman and readable bordered tables. The four-part setup/design/results/discussion outline is retained in seven pages.

`../../src/build_report.py` generates both formats from one content model. The retained document source is `../archive/lab_02_report_draft_20261020.docx`. Its opaque theme/settings/package parts remain unchanged. Body, styles and new media are editable; explicit style alignment supersedes the earlier draft's geometry.

The builder verifies source archive hashes, active statistics, event errors/ratios and saved displacement errors. Final QA includes every PDF and rendered Word page. Physical records, controller algorithms and historical results remain unchanged. Compiler and plotting intermediates use temporary folders and are removed automatically.

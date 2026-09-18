# TeX Environment & Package Provenance Note

## Standard Distribution Requirement
This manuscript is authored for standard **TeX Live 2022+** (Debian/Ubuntu packages `texlive-latex-base`, `texlive-latex-extra`, `texlive-science`, `texlive-fonts-recommended`, `texlive-bibtex-extra`) or **Overleaf (TeX Live 2022/2023/2024)**.

On any standard TeX Live system with `texlive-science` installed, `algorithm.sty` and `algorithmic.sty` resolve automatically from the system path (`/usr/share/texmf-dist/tex/latex/algorithms/`).

## Local Fallback Provenance (For Minimal Non-Root Environments)
If compiling in an environment where `texlive-science` cannot be installed via `sudo apt install texlive-science`, local copies of `algorithm.sty` and `algorithmic.sty` may be present in this directory as a temporary build shim.

- **Exact Upstream Source**: CTAN `algorithms` bundle (`macros/latex/contrib/algorithms.zip`), unpacked via `latex algorithms.ins`.
- **Version**: Version `2009/08/24 v0.1` by Peter Rogowski and Rogério Brito (the exact upstream version packaged into `texlive-science`).

## Clean-Up for Production Submission
Before uploading to IEEE Author Portal, IEEE Xplore, or Overleaf:
```bash
# Remove local shims so TeX Live resolves packages from system paths
rm -f new_paper/algorithm.sty new_paper/algorithmic.sty
```
This guarantees that typesetting adheres strictly to the official TeX Live distribution without local style drift.

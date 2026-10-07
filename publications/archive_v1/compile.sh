#!/bin/bash
set -e

echo "=========================================================="
echo "Compiling ARIA IEEE Journal Paper..."
echo "=========================================================="

if ! command -v pdflatex &> /dev/null; then
    echo "Error: pdflatex is not installed."
    echo "Install TeX Live: sudo apt install texlive-latex-base texlive-latex-extra texlive-fonts-recommended texlive-bibtex-extra"
    exit 1
fi

pdflatex -interaction=nonstopmode aria_journal_paper.tex
bibtex aria_journal_paper
pdflatex -interaction=nonstopmode aria_journal_paper.tex
pdflatex -interaction=nonstopmode aria_journal_paper.tex

echo "=========================================================="
echo "Compilation complete! Output: aria_journal_paper.pdf"
echo "=========================================================="

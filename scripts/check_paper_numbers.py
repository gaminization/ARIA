#!/usr/bin/env python3
"""
scripts/check_paper_numbers.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PROJECT ARIA: Publication Numerical Consistency & CSV Data Provenance Checker

Directive 7:
  Replaced legacy ban-list with direct CSV provenance validation.
  Checks every quantitative empirical result in main.tex against active CSV columns.
  Fails with exit code 1 on any unsourced or mismatched claim.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import os
import sys
from verify_paper_claims_against_csv import main as verify_main

if __name__ == "__main__":
    verify_main()


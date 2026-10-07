with open('aria_journal_paper.tex', 'r', encoding='utf-8') as f:
    text = f.read()

# 1. Fix line 824 typo: (}/wrist\_camera/image\_raw`) -> (\texttt{/wrist\_camera/image\_raw})
bad_chunk = "(}/wrist\\_camera/image\\_raw`)"
good_chunk = "(\\texttt{/wrist\\_camera/image\\_raw})"
if bad_chunk in text:
    text = text.replace(bad_chunk, good_chunk)
    print("Fixed line 824 camera topic typo!")

# 2. Table II -> table*
text = text.replace(
    "\\begin{table}[!t]\n\\renewcommand{\\arraystretch}{1.2}\n\\caption{ARIA 15-Agent Inventory",
    "\\begin{table*}[!t]\n\\renewcommand{\\arraystretch}{1.2}\n\\caption{ARIA 15-Agent Inventory"
)
table2_old_end = "\\bottomrule\n\\end{tabular}\n\\end{table}\n\n\n\\subsection{Dynamic Moving Target"
table2_new_end = "\\bottomrule\n\\end{tabular}\n\\end{table*}\n\n\n\\subsection{Dynamic Moving Target"
if table2_old_end in text:
    text = text.replace(table2_old_end, table2_new_end)
    print("Updated Table II to table*!")

# 3. Table V (10-Task VLA Benchmark) -> table*
text = text.replace(
    "\\begin{table}[!t]\n\\renewcommand{\\arraystretch}{1.2}\n\\caption{Standardized 10-Task VLA Benchmark",
    "\\begin{table*}[!t]\n\\renewcommand{\\arraystretch}{1.2}\n\\caption{Standardized 10-Task VLA Benchmark"
)
table5_old_end = "\\bottomrule\n\\end{tabular}\n\\end{table}\n\n\\begin{figure*}[!t]"
table5_new_end = "\\bottomrule\n\\end{tabular}\n\\end{table*}\n\n\\begin{figure*}[!t]"
if table5_old_end in text:
    text = text.replace(table5_old_end, table5_new_end)
    print("Updated Table V to table*!")

# 4. Table Interfaces -> table*
text = text.replace(
    "\\begin{table}[!h]\n\\renewcommand{\\arraystretch}{1.2}\n\\caption{ARIA ROS 2 Custom Service and Message Definitions}",
    "\\begin{table*}[!t]\n\\renewcommand{\\arraystretch}{1.2}\n\\caption{ARIA ROS 2 Custom Service and Message Definitions}"
)
table_int_old_end = "\\bottomrule\n\\end{tabular}\n\\end{table}\n\n% ==="
table_int_new_end = "\\bottomrule\n\\end{tabular}\n\\end{table*}\n\n% ==="
if table_int_old_end in text:
    text = text.replace(table_int_old_end, table_int_new_end)
    print("Updated Table Interfaces to table*!")

# 5. Add \footnotesize\setlength{\tabcolsep}{...} to single column tables
text = text.replace(
    "\\caption{Actuator & Physical Specifications of ARIA Manipulator}\n\\label{table_actuators}\n\\centering\n\\begin{tabular}{llcccc}",
    "\\caption{Actuator & Physical Specifications of ARIA Manipulator}\n\\label{table_actuators}\n\\centering\n\\footnotesize\n\\setlength{\\tabcolsep}{3pt}\n\\begin{tabular}{llcccc}"
)

text = text.replace(
    "\\caption{Dynamic VRAM Allocation Across Perception Modes}\n\\label{table_vram}\n\\centering\n\\begin{tabular}{llcc}",
    "\\caption{Dynamic VRAM Allocation Across Perception Modes}\n\\label{table_vram}\n\\centering\n\\footnotesize\n\\setlength{\\tabcolsep}{3pt}\n\\begin{tabular}{llcc}"
)

text = text.replace(
    "\\caption{Inverse Kinematics Solver Benchmark Results ($N=200$ Poses)}\n\\label{table_ik_benchmark}\n\\centering\n\\begin{tabular}{lcccc}",
    "\\caption{Inverse Kinematics Solver Benchmark Results ($N=200$ Poses)}\n\\label{table_ik_benchmark}\n\\centering\n\\footnotesize\n\\setlength{\\tabcolsep}{3pt}\n\\begin{tabular}{lcccc}"
)

text = text.replace(
    "\\caption{Perception & Metric Depth Reconstruction Metrics}\n\\label{table_perception}\n\\centering\n\\begin{tabular}{lccc}",
    "\\caption{Perception & Metric Depth Reconstruction Metrics}\n\\label{table_perception}\n\\centering\n\\footnotesize\n\\setlength{\\tabcolsep}{3.5pt}\n\\begin{tabular}{lccc}"
)

text = text.replace(
    "\\caption{End-to-End Computational Latency Breakdown}\n\\label{table_latency}\n\\centering\n\\begin{tabular}{lcc}",
    "\\caption{End-to-End Computational Latency Breakdown}\n\\label{table_latency}\n\\centering\n\\footnotesize\n\\setlength{\\tabcolsep}{3pt}\n\\begin{tabular}{lcc}"
)

# 6. Split line 916 in align
old_align = """\\begin{align}
r_w^2 + z_w^2 &= L_1^2 (\\cos^2\\theta_2 + \\sin^2\\theta_2) + L_2^2 (\\cos^2(\\theta_2+\\theta_3) + \\sin^2(\\theta_2+\\theta_3)) \\nonumber \\\\
&\\quad + 2 L_1 L_2 (\\cos\\theta_2 \\cos(\\theta_2+\\theta_3) + \\sin\\theta_2 \\sin(\\theta_2+\\theta_3)) \\nonumber \\\\
r_w^2 + z_w^2 &= L_1^2 + L_2^2 + 2 L_1 L_2 \\cos\\theta_3
\\end{align}"""

new_align = """\\begin{align}
r_w^2 + z_w^2 &= L_1^2 (\\cos^2\\theta_2 + \\sin^2\\theta_2) \\nonumber \\\\
&\\quad + L_2^2 (\\cos^2(\\theta_2+\\theta_3) + \\sin^2(\\theta_2+\\theta_3)) \\nonumber \\\\
&\\quad + 2 L_1 L_2 (\\cos\\theta_2 \\cos(\\theta_2+\\theta_3) + \\sin\\theta_2 \\sin(\\theta_2+\\theta_3)) \\nonumber \\\\
r_w^2 + z_w^2 &= L_1^2 + L_2^2 + 2 L_1 L_2 \\cos\\theta_3
\\end{align}"""

if old_align in text:
    text = text.replace(old_align, new_align)
    print("Split long equation line 916!")

with open('aria_journal_paper.tex', 'w', encoding='utf-8') as f:
    f.write(text)

print("Layout refinement complete!")

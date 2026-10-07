with open('aria_journal_paper.tex', 'r', encoding='utf-8') as f:
    text = f.read()

# Replace math environment \_ with _
math_envs = ['equation', 'align', 'equation*', 'align*', 'bmatrix', 'pmatrix', 'gather']

for env in math_envs:
    start_tag = f'\\begin{{{env}}}'
    end_tag = f'\\end{{{env}}}'
    
    parts = []
    curr = 0
    while True:
        s_idx = text.find(start_tag, curr)
        if s_idx == -1:
            parts.append(text[curr:])
            break
        e_idx = text.find(end_tag, s_idx)
        if e_idx == -1:
            parts.append(text[curr:])
            break
        
        parts.append(text[curr:s_idx])
        math_block = text[s_idx:e_idx + len(end_tag)]
        
        # Replace \_ with _ in math_block EXCEPT inside \text{...}
        # First protect \text{...}
        import re
        text_spans = []
        def save_text(m):
            text_spans.append(m.group(0))
            return f"__TEXT_SPAN_{len(text_spans)-1}__"
        
        protected = re.sub(r'\\text\{[^}]*\}', save_text, math_block)
        protected = protected.replace(r'\_', '_')
        for i, span in enumerate(text_spans):
            protected = protected.replace(f"__TEXT_SPAN_{i}__", span)
            
        parts.append(protected)
        curr = e_idx + len(end_tag)
    
    text = ''.join(parts)

# Also for inline math $...$
import re
def fix_inline(m):
    block = m.group(0)
    text_spans = []
    def save_text(tm):
        text_spans.append(tm.group(0))
        return f"__TEXT_SPAN_{len(text_spans)-1}__"
    protected = re.sub(r'\\text\{[^}]*\}', save_text, block)
    protected = protected.replace(r'\_', '_')
    for i, span in enumerate(text_spans):
        protected = protected.replace(f"__TEXT_SPAN_{i}__", span)
    return protected

text = re.sub(r'\$[^$\n]+\$', fix_inline, text)

with open('aria_journal_paper.tex', 'w', encoding='utf-8') as f:
    f.write(text)

print('Cleaned math blocks successfully!')

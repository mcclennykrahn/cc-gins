"""Build the JSON companion from the current audited Markdown table.

Never regenerate Markdown from obsolete audit snapshots or erase dated evidence.
"""
from collections import Counter
import json
from pathlib import Path
import re

rows = []
for line in Path('docs/proposal_alignment_matrix.md').read_text().splitlines():
    if not line.startswith('| ') or ' | ' not in line:
        continue
    parts = [x.strip() for x in line.strip('|').split('|')]
    if len(parts) != 5 or parts[3] not in ('complete', 'partial', 'experiment_substitute', 'unimplemented'):
        continue
    ident, _, proposal = parts[0].partition(' ')
    rows.append(dict(id=ident, proposal=proposal, expected=parts[1], reality=parts[2],
                     status=parts[3], evidence=re.findall(r'`([^`]+)`', parts[4])))
counts = dict(Counter(row['status'] for row in rows))
payload = dict(schema_version=1, audit_stage='fourth batch; causal learned streaming opt-in; prior measured evidence retained',
               counts=counts, items=rows)
Path('results/proposal_alignment/matrix.json').write_text(json.dumps(payload, indent=2, ensure_ascii=False)+'\n')
print(counts)

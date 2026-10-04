#!/usr/bin/env bash
# docs/TECHNICAL_REPORT.md -> docs/TECHNICAL_REPORT.pdf (pandoc + wkhtmltopdf); prints the page count.
set -euo pipefail
cd "$(dirname "$0")/.."
cat > /tmp/report.css <<'C'
body{font-family:Helvetica,Arial,sans-serif;font-size:9.2pt;line-height:1.32;max-width:none;margin:0}
h1{font-size:15pt;margin:0 0 4pt}h2{font-size:11.5pt;margin:10pt 0 3pt;border-bottom:1px solid #ccc}h3{font-size:10pt;margin:8pt 0 2pt}
table{border-collapse:collapse;font-size:7.6pt;margin:3pt 0}td,th{border:1px solid #bbb;padding:1.5pt 3pt}th{background:#f2f2f2}
pre,code{font-size:7.6pt}img{max-width:100%}p{margin:3pt 0}ul{margin:2pt 0 2pt 14pt;padding:0}li{margin:1pt 0}
C
pandoc docs/TECHNICAL_REPORT.md --resource-path=docs -s --metadata title=" " -c /tmp/report.css --self-contained -o /tmp/report.html
wkhtmltopdf -q --enable-local-file-access -s A4 -T 14mm -B 14mm -L 14mm -R 14mm /tmp/report.html docs/TECHNICAL_REPORT.pdf || true
python3 -c "import re;d=open('docs/TECHNICAL_REPORT.pdf','rb').read();print('pages:',len(re.findall(rb'/Type\s*/Page[^s]',d)))"

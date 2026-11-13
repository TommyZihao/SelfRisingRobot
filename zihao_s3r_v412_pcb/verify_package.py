# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import hashlib,json
p=Path(__file__).resolve().parent
checks=json.loads((p/'SHA256SUMS.json').read_text())
for name,digest in checks.items():
 f=p/name
 assert f.is_file(),name
 assert hashlib.sha256(f.read_bytes()).hexdigest()==digest,name
print(f'PASS: {len(checks)} packaged files match SHA256')

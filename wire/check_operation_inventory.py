#!/usr/bin/env python3
"""Validate the active typed operation vocabulary against reachable native dispatch."""
from __future__ import annotations
import argparse, hashlib, json, re, subprocess, sys
from pathlib import Path
import yaml

HANDLERS={"data":"Payload","simulation":"Simulation","render":"Render","viewer":"Viewer","streams":"Streams"}
CMP=re.compile(r'(?:Request\.Op|Completion->Request\.Op|\bOp)\s*[!=]=\s*TEXT\("([a-z][a-z0-9_]*)"\)')

def fail(errors):
 print("operation inventory check failed:",file=sys.stderr); print("\n".join("  - "+e for e in errors),file=sys.stderr); return 1

def main()->int:
 ap=argparse.ArgumentParser(description=__doc__); ap.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]); ap.add_argument('--skip-generated-check',action='store_true'); a=ap.parse_args()
 root=a.root.resolve(); plugin=root/'UnrealRoboticsLab' if (root/'UnrealRoboticsLab').is_dir() else root; workspace=plugin.parent
 wire=plugin/'wire'; schema_path=wire/'urlab_wire.yaml'; generated=plugin/'Source/URLab/Public/Wire/UrlabWire.gen.h'; bridge_generated=workspace/'URLab_Bridge/src/urlab_client/_wire/generated.py'
 errors=[]; schema=yaml.safe_load(schema_path.read_text()); messages=schema['messages']
 active={v['op']:v for v in messages.values() if isinstance(v,dict) and v.get('typed_dispatch')=='active'}
 lifecycle={v['op']:v for v in messages.values() if isinstance(v,dict) and v.get('typed_dispatch')=='lifecycle'}
 classified=set(active)|set(lifecycle)
 unclassified={v['op'] for v in messages.values() if isinstance(v,dict) and isinstance(v.get('op'),str) and v.get('op') and v['op'] not in classified}
 for op in sorted(unclassified): errors.append(f"{op}: schema operation lacks active/lifecycle typed_dispatch classification")
 if set(lifecycle)!={'connect','disconnect'}: errors.append(f"lifecycle operations are {sorted(lifecycle)}, expected connect/disconnect")
 for op,d in active.items():
  for field in ('service','permission','authority','execution','adapters'):
   if field not in d or d[field] is None or d[field]=='' or d[field]==[]: errors.append(f"{op}: missing explicit {field} policy")
  if d.get('permission') is None or d.get('authority') is None: errors.append(f"{op}: use explicit 'none', not null, for absent policy")
 # Generated freshness is delegated to the canonical generator; metadata below is independently parsed.
 if not a.skip_generated_check:
  p=subprocess.run([sys.executable,str(wire/'gen_wire.py'),'--check'],cwd=workspace,text=True,capture_output=True)
  if p.returncode: errors.append("generated outputs stale: "+(p.stderr or p.stdout).strip().splitlines()[0])
 h=generated.read_text(); py=bridge_generated.read_text(); digest=hashlib.sha256(schema_path.read_bytes()).hexdigest()
 for label,text,pat in [('C++',h,r'SchemaSha256 = TEXT\("([0-9a-f]+)"\)'),('Python',py,r'SCHEMA_SHA256 = "([0-9a-f]+)"')]:
  m=re.search(pat,text)
  if not m or m.group(1)!=digest: errors.append(f"{label} generated schema SHA is stale")
 arrays={name:re.findall(r'TEXT\("([^\"]+)"\)',body) for name,body in re.findall(r'inline const TCHAR\* const (\w+)Adapters\[\] = \{(.*?)\};',h,re.S)}
 block=h[h.index('namespace TypedOp'):]; block=block[block.index('inline const FDecl All[]'):block.index('inline constexpr int32 Count',block.index('inline const FDecl All[]'))]
 rows={}
 for m in re.finditer(r'\{TEXT\("([^\"]+)"\), TEXT\("([^\"]+)"\), TEXT\("([^\"]+)"\), TEXT\("([^\"]+)"\), TEXT\("([^\"]+)"\), TEXT\("([^\"]+)"\), (\w+)Adapters, (\d+) \}',block):
  op,shape,service,perm,auth,execution,arr,count=m.groups(); rows[op]=(shape,service,perm,auth,execution,arrays.get(arr,[]),int(count))
 if set(rows)!=set(active):
  for op in sorted(set(active)-set(rows)): errors.append(f"{op}: absent from generated TypedOp declarations")
  for op in sorted(set(rows)-set(active)): errors.append(f"{op}: generated TypedOp has no active schema operation")
 for op,d in active.items():
  if op not in rows: continue
  expected=(d['request_shape'],str(d['service']),str(d['permission']),str(d['authority']),str(d['execution']),list(d['adapters']),len(d['adapters']))
  if rows[op]!=expected: errors.append(f"{op}: generated policy metadata differs from schema")
 connection=(plugin/'Source/URLab/Private/Bridge/RpcHandlers_TypedConnection.cpp').read_text()
 for op in lifecycle:
  if not re.search(rf'\bOp\s*==\s*TEXT\("{op}"\)',connection): errors.append(f"{op}: lifecycle branch is not reachable")
 for hook in ('URLabWire::TypedOp::Find','URLabWire::OpAdapter::Allows','FURLabOperationSchema::ValidateRequestBody'):
  if hook not in connection: errors.append(f"typed connection lacks policy hook {hook}")
 # Extract actual service/execution routing lanes and require each declared lane to reach its service dispatcher.
 route_blocks=re.split(r'(?=\n\s*if \(Service == TEXT\()' ,connection)
 routes=set()
 for b in route_blocks:
  sm=re.search(r'Service == TEXT\("([^\"]+)"\)',b); dm=re.search(r'DispatchTyped(\w+)\(',b)
  if not sm or not dm: continue
  for execution in re.findall(r'Execution == TEXT\("([^\"]+)"\)',b): routes.add((sm.group(1),execution,dm.group(1)))
 for op,d in active.items():
  target='Editor' if d['service']=='editor' else HANDLERS.get(d['service'])
  if (d['service'],d['execution'],target) not in routes: errors.append(f"{op}: no reachable {d['service']}/{d['execution']} -> DispatchTyped{target} lane")
 # Concrete handler comparisons are a second, independent vocabulary. Exactly one implicit final else is tolerated only when unambiguous.
 reachable=set()
 for service,suffix in HANDLERS.items():
  text=(plugin/f'Source/URLab/Private/Bridge/RpcHandlers_Typed{suffix}.cpp').read_text(); actual=set(CMP.findall(text)); expected={op for op,d in active.items() if d['service']==service}; extra=actual-set(active)
  for op in sorted(extra): errors.append(f"{op}: orphan reachable comparison in Typed{suffix}")
  missing=expected-actual
  if service=='streams' and len(missing)==1 and re.search(r'\n\s*else\s*\{',text): reachable|=missing
  else:
   for op in sorted(missing): errors.append(f"{op}: active schema op is not handled by Typed{suffix}")
  reachable|=actual&expected
 editor=(plugin/'Source/URLabEditor/Private/MjEditorOpHandlers.cpp').read_text(); registered=set(re.findall(r'\bRegEditor\(TEXT\("([^\"]+)"\)',editor)); expected_editor={op[7:] for op,d in active.items() if d['service']=='editor'}
 for op in sorted(registered-expected_editor): errors.append(f"editor_{op}: orphan RegEditor registration")
 for op in sorted(expected_editor-registered): errors.append(f"editor_{op}: missing RegEditor registration")
 adapter=(plugin/'Source/URLab/Private/Bridge/RpcHandlers_TypedEditor.cpp').read_text()
 if 'Request.Op.Mid(7)' not in adapter or 'TEXT("__typed_editor_") + Legacy' not in adapter: errors.append("typed editor prefix adapter is missing")
 reachable|={'editor_'+x for x in registered&expected_editor}
 inv=json.loads((wire/'operation_inventory.json').read_text())
 retired={x['op'] for x in inv.get('retired_operations',[])}
 vocab=set(active)|set(lifecycle)|set(rows)|reachable|set(re.findall(r'^OP_[A-Z0-9_]+ = "([^\"]+)"',py,re.M))
 for op in sorted(retired&vocab): errors.append(f"retired operation reintroduced: {op}")
 if errors:return fail(errors)
 print(f"operation inventory check passed: {len(active)} active, {len(lifecycle)} lifecycle, {len(retired)} retired absent")
 return 0
if __name__=='__main__': raise SystemExit(main())

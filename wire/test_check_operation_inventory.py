"""Mutation tests for typed operation reachability validation."""
from __future__ import annotations
import shutil, subprocess, sys, tempfile, unittest
from pathlib import Path
import yaml
HERE=Path(__file__).resolve().parent; PLUGIN=HERE.parent; WORKSPACE=PLUGIN.parent

class OperationInventoryCheckTest(unittest.TestCase):
 def fixture(self):
  holder=tempfile.TemporaryDirectory(); self.addCleanup(holder.cleanup); root=Path(holder.name); plugin=root/'UnrealRoboticsLab'; bridge=root/'URLab_Bridge'
  for rel in ('wire/check_operation_inventory.py','wire/operation_inventory.json','wire/urlab_wire.yaml','Source/URLab/Public/Wire/UrlabWire.gen.h','Source/URLab/Private/Bridge/RpcHandlers_TypedConnection.cpp','Source/URLab/Private/Bridge/RpcHandlers_TypedPayload.cpp','Source/URLab/Private/Bridge/RpcHandlers_TypedSimulation.cpp','Source/URLab/Private/Bridge/RpcHandlers_TypedRender.cpp','Source/URLab/Private/Bridge/RpcHandlers_TypedViewer.cpp','Source/URLab/Private/Bridge/RpcHandlers_TypedStreams.cpp','Source/URLab/Private/Bridge/RpcHandlers_TypedEditor.cpp','Source/URLabEditor/Private/MjEditorOpHandlers.cpp'):
   dst=plugin/rel; dst.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(PLUGIN/rel,dst)
  dst=bridge/'src/urlab_client/_wire/generated.py'; dst.parent.mkdir(parents=True); shutil.copy2(WORKSPACE/'URLab_Bridge/src/urlab_client/_wire/generated.py',dst)
  return root,plugin
 def check(self,root): return subprocess.run([sys.executable,str(root/'UnrealRoboticsLab/wire/check_operation_inventory.py'),'--root',str(root),'--skip-generated-check'],text=True,capture_output=True)
 def mutate(self,path,old,new):
  text=path.read_text(); self.assertIn(old,text); path.write_text(text.replace(old,new,1))
 def test_actual_snapshot_passes(self):
  root,_=self.fixture(); self.assertEqual(self.check(root).returncode,0)
 def test_missing_and_orphan_handler_comparison_fail(self):
  root,p=self.fixture(); f=p/'Source/URLab/Private/Bridge/RpcHandlers_TypedSimulation.cpp'; f.write_text(f.read_text().replace('TEXT("simulation_describe")','TEXT("rogue_describe")')); r=self.check(root); self.assertNotEqual(r.returncode,0); self.assertIn('orphan reachable comparison',r.stderr); self.assertIn('simulation_describe: active schema op is not handled',r.stderr)
 def test_route_dispatch_mutation_fails(self):
  root,p=self.fixture(); f=p/'Source/URLab/Private/Bridge/RpcHandlers_TypedConnection.cpp'; self.mutate(f,'return DispatchTypedRender(Request','return DispatchTypedViewer(Request'); r=self.check(root); self.assertNotEqual(r.returncode,0); self.assertIn('no reachable render/render_queue',r.stderr)
 def test_unclassified_schema_operation_fails(self):
  root,p=self.fixture(); f=p/'wire/urlab_wire.yaml'; data=yaml.safe_load(f.read_text()); data['messages']['render_scene_describe'].pop('typed_dispatch'); f.write_text(yaml.safe_dump(data,sort_keys=False)); r=self.check(root); self.assertNotEqual(r.returncode,0); self.assertIn('render_scene_describe: schema operation lacks active/lifecycle',r.stderr)
 def test_schema_adapter_metadata_drift_fails(self):
  root,p=self.fixture(); f=p/'wire/urlab_wire.yaml'; data=yaml.safe_load(f.read_text()); data['messages']['render_scene_describe']['adapters']=['zmq']; f.write_text(yaml.safe_dump(data,sort_keys=False)); r=self.check(root); self.assertNotEqual(r.returncode,0); self.assertIn('generated policy metadata differs',r.stderr)
 def test_generated_sha_staleness_fails(self):
  root,_=self.fixture(); f=root/'URLab_Bridge/src/urlab_client/_wire/generated.py'; self.mutate(f,'SCHEMA_SHA256 = "','SCHEMA_SHA256 = "0'); r=self.check(root); self.assertNotEqual(r.returncode,0); self.assertIn('Python generated schema SHA is stale',r.stderr)
 def test_editor_wrapper_mismatch_fails(self):
  root,p=self.fixture(); f=p/'Source/URLabEditor/Private/MjEditorOpHandlers.cpp'; self.mutate(f,'RegEditor(TEXT("current_level")','RegEditor(TEXT("retired_level")'); r=self.check(root); self.assertNotEqual(r.returncode,0); self.assertIn('orphan RegEditor registration',r.stderr); self.assertIn('missing RegEditor registration',r.stderr)
 def test_retired_op_reintroduction_fails(self):
  root,_=self.fixture(); f=root/'URLab_Bridge/src/urlab_client/_wire/generated.py'; f.write_text(f.read_text()+'\nOP_RETIRED = "render_hello"\n'); r=self.check(root); self.assertNotEqual(r.returncode,0); self.assertIn('retired operation reintroduced: render_hello',r.stderr)
if __name__=='__main__': unittest.main()

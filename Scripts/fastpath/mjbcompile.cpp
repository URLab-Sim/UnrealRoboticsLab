// mjbcompile: compile a MuJoCo scene.xml into a version-matched .mjb.
//
// The fast-path renderer (AMjbScene) loads an MJB with the plugin's libmujoco,
// which is version-locked (mj_loadModelBuffer rejects a mismatched version). A
// fast-path OWNER must therefore serve an MJB compiled by the SAME libmujoco the
// plugin ships. This tiny tool does that: link it against
// third_party/install/MuJoCo and run it to turn an XML into an MJB.
//
// Build + run recipe: see UnrealRoboticsLab/docs/fast_path_render.md
// (UE clang + static libc++/libc++abi + --allow-shlib-undefined; run with the
// system libdir ahead of libmujoco's stale RUNPATH).
#include <mujoco/mujoco.h>
#include <stdio.h>

int main(int argc, char** argv) {
  if (argc < 3) { fprintf(stderr, "usage: %s in.xml out.mjb\n", argv[0]); return 2; }
  char err[1024] = {0};
  mjModel* m = mj_loadXML(argv[1], NULL, err, sizeof(err));
  if (!m) { fprintf(stderr, "loadXML failed: %s\n", err); return 1; }
  mj_saveModel(m, argv[2], NULL, 0);
  printf("ok: %s -> %s  (nbody=%d ngeom=%d nq=%d nmesh=%d ver=%d)\n",
         argv[1], argv[2], (int)m->nbody, (int)m->ngeom, m->nq, (int)m->nmesh, mj_version());
  mj_deleteModel(m);
  return 0;
}

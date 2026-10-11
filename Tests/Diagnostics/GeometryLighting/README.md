# Geometry lighting GPU diagnostic

`RenderGeometryLightingGpuTests.cpp` is a preserved diagnostic fixture, outside
the compiled editor module. Its first run on 2026-09-23 reached the assertions,
but all nine front/back lighting checks and the box shadow check failed. The
cause is undetermined; this file is not a passing regression test or part of the
normal test suite. The run log is `/tmp/urlab_bunny_allgeom_gpu_override.log` in
the local workspace. The user accepted the live bunny, primitive, mesh, and
textured robot scene by visual inspection, which does not resolve these fixture
failures.

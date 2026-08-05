mujoco_version "3.10.0"

enum Mode { a = "a"  b = "b" }

element Sample {
  scalar_i  : int32
  scalar_u  : uint64
  scalar_f  : float
  scalar_d  : double
  flag      : bool
  label     : string
  fixed3    : double[3]
  range03   : double[0..3]
  range13   : double[1..3] = {1, 0.005, 0.0001}
  unbounded : double[]
  color     : float[4] = {0.5, 0.5, 0.5, 1}
  neg       : double = -1.5
  count     : int32 = 7
  on        : bool = true
  mode      : Mode = a
  keywords  : Mode[]
}

mujoco_version "3.10.0"

enum Integrator {
  euler        = "Euler"
  implicit     = "implicit"
  implicitfast = "implicitfast"
  rk4          = "RK4"
}

enum GeomType {
  sphere  = "sphere"
  capsule = "capsule"
  box     = "box"
  mesh    = "mesh"
}

struct Quat      { w : double  x : double  y : double  z : double }
struct Euler     { angles : double[3] (unit=angle) }
struct AxisAngle { axis : double[3]  angle : double (unit=angle) }

variant Orientation {
  quat      : Quat        # unit quaternion (canonical form)
  euler     : Euler       # euler angles, sequence from compiler.eulerseq
  axisangle : AxisAngle
}

struct Explicit { size : double[0..3] }
struct FromTo   { fromto : double[6] }

variant GeomShape {
  explicit : Explicit
  fromto   : FromTo
}

struct Diagonal { inertia : double[3] }
struct Full     { fullinertia : double[6] }

variant InertiaSpec {
  diagonal : Diagonal
  full     : Full
}

element Material { name : string }
element Mesh     { name : string }
element Default  { name : string }

mixin Posed {
  pos    : double[3] = {0, 0, 0}       # position offset
  orient : variant Orientation         # quat | euler | axisangle
}

element Geom {
  use Posed
  name     : string
  dclass   : ref<Default>  (xml="class")     # `class` is reserved in C++
  type     : GeomType = sphere
  size     : double[0..3]
  shape    : variant GeomShape
  friction : double[1..3] = {1, 0.005, 0.0001}
  material : ref<Material>
  mesh     : ref<Mesh>
  rgba     : float[4] = {0.5, 0.5, 0.5, 1}
  user     : double[]
}

element Frame {
  use Posed
  name : string
}

element Joint {
  name : string
  type : string (required)             # structural: hinge | slide | ball | free
}

element Body {
  use Posed
  name       : string
  childclass : ref<Default>
  inertial   : variant InertiaSpec
  children geoms  : Geom  *
  children joints : Joint *
  children bodies : Body  *
  children frames : Frame *
}

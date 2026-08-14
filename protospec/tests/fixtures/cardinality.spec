mujoco_version "3.10.0"

element Leaf { name : string }

element Root {
  name : string
  children many     : Leaf *
  children optional : Leaf ?
  children exactly  : Leaf !
}

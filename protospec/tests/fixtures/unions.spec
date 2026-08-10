mujoco_version "3.10.0"

element Motor    { name : string }
element Position { name : string }
element Plugin (xml="plugin") { name : string }

union ActuatorAny = Motor | Position | Plugin

element Actuator {
  children items : ActuatorAny *
}

element Transmission {
  name   : string
  target : ref<ActuatorAny>          # any member's namespace
  children items : ActuatorAny ?
}

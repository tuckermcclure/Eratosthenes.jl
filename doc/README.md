Model Interfaces
================

Body
----

```
```

Sensor
------

```
init(t, constants, state, draws, vehicle_state)
sense(t, constants, state, draws, vehicle_state, effects, vehicle_states)
step(t, constants, state, draws, vehicle_state, effects, vehicle_states)
```

Actuator
---------

```
init(t, constants, state, draws, vehicle_state)
effects(t, constants, state, draws, command, vehicle_state, vehicle_states)
sense(t, constants, state, draws, vehicle_state, effects, vehicle_states)
step(t, constants, state, draws, vehicle_state, effects, vehicle_states)
```

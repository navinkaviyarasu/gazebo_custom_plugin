# gazebo_custom_plugin

## Compiling & Running

### Building the plugin
```bash
mkdir build && cd build
cmake ..
make
```

### Adding Gazebo custom plugin path

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)
```

## List of Plugins

### libTVCPlugin

#### TVC Plugin SDF Parameters

The model includes a plugin block that loads the TVC simulation plugin (`libTVCPlugin.so`). The plugin accepts the following SDF elements (defaults shown):

- `<dynamics_model>`: order of the system: 1st or 2nd. Default:`first_order`. Options: `first_order` or `second_order`
- `<servo_time_constant>`: time constant in seconds, first order parameter
- `<servo_natural_freq>`: natural frequency in rad/sec, second order parameter
- `<servo_damping_ratio>`: damping ratio, second order parameter 

- `<roll_joint>`: name of the roll joint (string). Default: `servo0_roll_joint`.
- `<pitch_joint>`: name of the pitch joint (string). Default: `servo1_pitch_joint`.
- `<engine_link>`: link name to apply thrust/attach propellers (string). Default: `motor_housing`.
- `<topic>`: Gazebo transport topic for servo length commands (string). Default: `/tvc_servo_command`.
- `<servo_max_speed>`: maximum linear speed of the servo (m/s). Default: `0.112`.
- `<servo_time_constant>`: first-order time constant (seconds) for servo dynamics (tau). If `<= 0` the plugin falls back to a rate-limiter. Default: `0.10`.
- `<servo_min_len>`: minimum physical servo length (m). Default: `0.085`.
- `<servo_max_len>`: maximum physical servo length (m). Default: `0.115`.

Notes:
- The plugin maps two commanded servo lengths into roll/pitch joint positions using a simplified kinematic model. Geometry (anchor/clevis points) is currently hard-coded in the plugin source.
- If you change the geometry in `model.sdf`, update the plugin constants or (preferably) extend the plugin to read link poses and compute lengths dynamically.

#### Example plugin block (from `model.sdf`)

```xml
    <plugin filename="libTVCPlugin.so" name="tvc_plugin::TVCPlugin">
      <!-- Dynamics Model Selection: "first_order" or "second_order" -->
      <dynamics_model>first_order</dynamics_model>
      
      <!-- Common Parameters -->
      <servo_max_speed>0.112</servo_max_speed>
      <servo_min_len>0.085</servo_min_len>
      <servo_max_len>0.115</servo_max_len>
      
      <!-- First-order System Parameters -->
      <!-- Time constant (seconds) for first-order model -->
      <servo_time_constant>0.1</servo_time_constant>
      
      <!-- Second-order System Parameters -->
      <!-- Natural frequency (rad/s) for second-order model - typical range 2-10 -->
      <servo_natural_freq>5.0</servo_natural_freq>
      <!-- Damping ratio (dimensionless) - 0.7 is slightly underdamped with minimal overshoot -->
      <servo_damping_ratio>0.7</servo_damping_ratio>
      
      <!-- Joint/Link References -->
      <roll_joint>servo0_roll_joint</roll_joint>
      <pitch_joint>servo1_pitch_joint</pitch_joint>
      <engine_link>motor_housing</engine_link>
      
      <!-- Gazebo Transport Topic for Servo Commands -->
      <topic>/tvc_servo_command</topic>
    </plugin>
```

If you need the plugin to use different limits or dynamics, change the SDF values or update the plugin source in `TVCPlugin.cc`.

---
Last updated: 2026-03-23
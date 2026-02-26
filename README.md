# STM32 BLDC Motor Controller

> This is a repo for VBDrive firmware
>
> [Buy here]()

## UART Configuration / Test Interface

The board uses a UART-based serial interface for configuration, calibration, test control, and debug logging.

### **Connection Details**

* **Baud Rate**: 19200
* **Format**: ASCII commands; trailing `\r`, `\n`, spaces and tabs are stripped automatically

---

### **Command Syntax**

* **Read parameter**:
  `<parameter_name>:?`
  Example: `node_id:?` -> `node_id:11`

* **Write parameter**:
  `<parameter_name>:<value>`
  Example: `kp:0.35` -> `OK: kp :0.350000`

---

### **Mode and Command Overview**

| Command | Available State | Description |
| ------- | --------------- | ----------- |
| `CONFIG` | Any non-TEST state | Enter configuration mode and stop motor |
| `SAVE` | CONFIG | Persist updated config to EEPROM (if changed), exit config mode, start motor |
| `RESET` | CONFIG | Load default config values in RAM (does NOT affect current session - requires `SAVE` or `APPLY` to persist) |
| `APPLY` | Any non-TEST state | Persist updated config (if changed) and reboot |
| `CALIBRATE` | RUNNING, NOT_CALIBRATED | Run calibration action |
| `TEST` | RUNNING | Enter test mode |
| `STOP` | TEST | Exit test mode, clear FOC target, stop test logging |

---

### **Configuration Parameters**

| Parameter      | Description                                         | Type    | Example Values |
| -------------- | --------------------------------------------------- | ------- | -------------- |
| `gear_ratio`   | Gear ratio of the drive                             | Integer | `1`, `5`, `15` |
| `max_current`  | Maximum motor current (A)                           | Float   | `5.0`, `10.5`  |
| `max_speed`    | Maximum motor speed (rad)                           | Float   | `1000.0`       |
| `max_torque`   | Maximum torque output (Nm)                          | Float   | `1.2`          |
| `angle_offset` | Motor angle offset (rad)                            | Float   | `0.0`, `15.5`  |
| `min_angle`    | Minimum allowed angle (rad)                         | Float   | `-30.0`        |
| `max_angle`    | Maximum allowed angle (rad)                         | Float   | `30.0`         |
| `torque_const` | Torque constant (Nm/A)                              | Float   | `0.12`         |
| `kp`           | Current proportional gain                           | Float   | `0.25`         |
| `ki`           | Current integral gain                               | Float   | `0.01`         |
| `kd`           | Current derivative gain                             | Float   | `0.005`        |
| `filter_a`     | Main filter parameter A                             | Float   | `0.5`          |
| `filter_g1`    | Filter gain 1                                       | Float   | `0.1`          |
| `filter_g2`    | Filter gain 2                                       | Float   | `0.1`          |
| `filter_g3`    | Filter gain 3                                       | Float   | `0.1`          |
| `I_lpf`        | Current low-pass filter coefficient                 | Float   | `0.1`          |
| `angle_encoder`| Angle encoder type enum, 0 rotor, 1 - shaft         | Integer | `0`, `1`.      |
| `node_id`      | Cyphal/CAN node ID                                  | Integer | `1`, `42`      |
| `data_baud`    | FDCAN data baud rate enum (see below)               | Enum    | `0`, `1`, `2`  |
| `nominal_baud` | FDCAN nominal baud rate enum (see below)            | Enum    | `3`, `4`       |

---

### **FDCAN Baud Rate Configuration**

| parameter           | Value Name | Speed    | Numeric Value |
|---------------------|------------|----------|---------------|
| `nominal_baud`      | `KHz62`    | 62.5 kHz | `0`           |
|                     | `KHz125`   | 125 kHz  | `1`           |
|                     | `KHz250`   | 250 kHz  | `2`           |
|                     | `KHz500`   | 500 kHz  | `3`           |
|                     | `KHz1000`  | 1 MHz    | `4`           |
|---------------------|------------|----------|---------------|
| `data_baud`         | `KHz1000`  | 1 MHz    | `0`           |
|                     | `KHz2000`  | 2 MHz    | `1`           |
|                     | `KHz4000`  | 4 MHz    | `2`           |
|                     | `KHz8000`  | 8 MHz    | `3`           |

---

### **TEST Mode Commands**

| Command | Description |
| ------- | ----------- |
| `do.velocity:<value>` | Set velocity target for FOC test controller |
| `do.angle:<value>` | Set angle target for FOC test controller |
| `do.free` | Zero target (no effort mode) |
| `min_angle:<value or ?>` | Read/write lower position limit during test |
| `max_angle:<value or ?>` | Read/write upper position limit during test |
| `angle_offset:<value or ?>` | Read/write angle offset during test |
| `log.start` | Start UART test logging |
| `log.stop` | Stop UART test logging |
| `STOP` | Exit test mode |

When logging is enabled in TEST mode, UART periodically prints:

* `rotor sensor: <u16>`
* `shaft sensor: <u16>`
* `shaft angle : <float>`

---

### **Response Format**

* **Success**: `OK: <param>:<value>` or `OK: <param> :<value>` (set operations)
* **Error**: `ERROR: Unknown command`, `ERROR: Unknown parameter`, `ERROR: Invalid value`
* **Config persistence**: Settings are written to EEPROM on `SAVE`/`APPLY` (not on every `SET`)

---

### **Example Sessions**

```bash
# Configuration flow
> CONFIG
CONFIG MODE ENABLED
> node_id:11
OK: node_id:11
> gear_ratio:36
OK: gear_ratio:36
> SAVE
Saved config
NOTE: config changes not applied! To apply, run APPLY or reset controller
> APPLY
```

```bash
# Test flow
> TEST
Entering TEST mode
> do.velocity:5
Set velocity: <5.000000>
> log.start
# periodic sensor logs...
> STOP
Stopping TEST mode
```

## FDCAN Cyphal Runtime Interface

The BLDC Motor Controller communicates over **Cyphal/FDCAN** to publish real-time telemetry and receive control commands.

> We use some custom datatypes, see here: [VoltBro cyphal types repository](https://github.com/voltbro/cyphal-types)

### **Published Messages**

| Port ID | Message Type                              | Interval | Description                                   |
| ------- | ----------------------------------------- | -------- | --------------------------------------------- |
| `3811`  | `voltbro.foc.state_simple.1.0`            | 1 ms     | Current state (angle, speed, torque, etc.)    |

---

### **Subscribed Messages**

| Port ID Formula  | Message Type                       | Description                                                                 |
| ---------------- | ---------------------------------- | --------------------------------------------------------------------------- |
| `2107 + node_id` | `voltbro.foc.command.1.0`          | Direct FOC target command: torque, angle, velocity, and PID gains           |
| `3407 + node_id` | `voltbro.foc.specific_control.1.0` | High-level, single parameter control setpoint                               |

---

### **Registers**

| Register Name | Type   | Description                                         |
| ------------- | ------ | --------------------------------------------------- |
| `motor.is_on` | `bool` | Turns underlying motor driver on/off                |

---

### **Standard Cyphal Services**

The controller also supports standard Cyphal services:

* **uavcan.node.GetInfo** — Reports node information (name: `"org.voltbro.vbdrive"`)
* **uavcan.register.Access** — For reading/writing runtime parameters
* **uavcan.register.List** — List registers
* **uavcan.node.Heartbeat** — Node status monitoring

### **Standard Cyphal Messages**

The controller also publishes standard Cyphal messages:

* **uavcan.node.Heartbeat** - default heartbeat message

---

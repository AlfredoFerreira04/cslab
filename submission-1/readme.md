# CSLAB - First Submission

## Requirement Specification

### Black Box

#### Requirements

| REQ. ID | TITLE | DESCRIPTION |
| :--- | :--- | :--- |
| **1** | **Automatic Natural Light Optimization** | While the system is in Automatic Mode, the ACCS shall adjust the smart blinds to the open position to maximize natural sunlight entry. |
| **2** | **Artificial Light Compensation** | When the natural light level, at its maximum, is measured below the user-defined threshold, the ACCS shall activate the artificial smart lights to compensate. |
| **3** | **Automatic Temperature Regulation** | While the system is in Automatic Mode, the ACCS shall modulate the smart heater output to maintain the office temperature within user preferences. |
| **4** | **User Preference Configuration** | When the Occupant inputs configuration changes via the User Interface, the ACCS shall store the new preferences for lighting and temperature. |
| **5** | **Manual Override Control** | When a manual command is received for a specific device (Blinds/Lights/Heater), the ACCS shall override the automatic control and execute the requested status change. |
| **6** | **Performance Deviation Alert** | If the measured environmental parameters deviate from the current system configuration target values for a defined duration, the ACCS shall display a warning message on the User Interface. |

#### System Context

* Components:
  * Automated Climate Control System (ACCS) [System of Interest]
  * Occupant
  * Physical Environment
  * Maintenance Services

![CISTER Office Environment](./diagrams/system-context.png)

#### Use Cases

//TODO

---

### White Box

#### Functional Decomposition

//TODO

#### Conceptual Interfaces

//TODO

#### Conceptual Subsystems

//TODO

## Selected Technology

//TODO

## Physical Sensor/Actuators

//TODO

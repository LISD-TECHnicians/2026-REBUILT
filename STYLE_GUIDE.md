# Team 5144 FRC 2026 Code Style Guide

This document outlines the coding style for all Java files. Following this standard ensures our code is readable, maintainable, and predictable.

---

## 1. Naming Conventions

We use a modified Hungarian Notation to make variable **scope** (where the variable lives) immediately obvious.

| Type | Convention | Prefix | Example |
| :--- | :--- | :--- | :--- |
| **Classes** | PascalCase | None | `DriveSubsystem` |
| **Constants** | camelCase | `k` | `kEncoderResolution` |
| **Member Variables** | camelCase | `m_` | `m_leftMasterMotor` |
| **Local Variables** | camelCase | None | `currentError` |
| **Methods** | camelCase | None | `getHeading()` |

### Scope Rules
* **The `m_` Rule:** Use `m_` **only** for variables declared at the top of the class. These "persist" as long as the robot is on.
* **The `k` Rule:** Use `k` for any fixed value (usually found in `Constants.java`).
* **Local Scope:** Variables created inside a method or constructor should have **no prefix**. They are deleted once the method finishes.

---

## 2. File Organization

Every `.java` file must follow this vertical order. **Do not hunt for code; know where it lives.**

### I. Header & Imports
1. **Package Statement**
2. **Imports:** Grouped by (Java → WPILib → Vendor → Team). Use a blank line between groups.

### II. Class Body
3. **Hardware Objects:** `private final` motor controllers, sensors, and gyros.
4. **State Variables:** `private` primitives (bools, doubles) tracking robot status.
5. **Constructor:** Initialization, factory resets, and hardware configuration.
6. **Periodic:** (`@Override`) Telemetry updates and sensor-to-state logic.
7. **Public API:** Verbs! Methods for Commands to call (e.g., `setSpeed()`).
8. **Private Helpers:** Internal math or logic used by the API.

---

## 3. Implementation Template

```java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ClimberConstants;

/**
 * ClimberSubsystem handles the winch and extension logic.
 */
public class ClimberSubsystem extends SubsystemBase {
    
    // --- 3. HARDWARE ---
    private final CANSparkMax m_motor = new CANSparkMax(ClimberConstants.kID, MotorType.kBrushless);

    // --- 4. STATE ---
    private boolean m_isExpanded = false;

    // --- 5. CONSTRUCTOR ---
    public ClimberSubsystem() {
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(ClimberConstants.kInverted);
    }

    // --- 6. PERIODIC ---
    @Override
    public void periodic() {
        // State updates
        m_isExpanded = (m_motor.getEncoder().getPosition() > 10.0);
    }

    // --- 7. PUBLIC API ---
    public void setClimbPower(double power) {
        m_motor.set(power);
    }

    public boolean isExtended() {
        return m_isExpanded;
    }

    // --- 8. PRIVATE HELPERS ---
    private double applyDeadband(double input) {
        return (Math.abs(input) < 0.05) ? 0.0 : input;
    }
}

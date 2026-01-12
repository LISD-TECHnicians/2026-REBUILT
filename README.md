# Team 5144 LISD TECHnicians 
## FRC 2026 Season - REBUILT
### Robot: [Robot Name Here]
---

**Welcome to the official repository for Team 5144's 2026 FRC Season. This project is built using the WPILib Command-Based framework in Java.**

## Table of Contents
* [Season Overview](#season-overview)
* [Robot Specifications](#robot-specifications)
* [Source Code Organization & Style](#source-code-organization--style)
* [Git Workflow](#️git-workflow)
* [Controls Map](#controls-map)

---

## Season Overview
 **Game:** REBUILT  
 **Objective:** Our robot is designed to be a _______ focused robot that prioritizes _______ in order to maximize scored points. Some of the highlights of out robot includes its ability to _____ and _____.

---

## Robot Specifications
| System | Description |
| :--- | :--- |
| **Drive Train** | Swerve based drivetrain utilizing Mki4 modules from Swerve Drive Specialties |
| **Processor** | RoboRIO 2.0 |
| **Vision** | (2) Limelight vision systems |
| **Motors** | CTRE: Kraken X60 / Falcon 500 |
| **Scoring Apparatus** | Ball turret with 360 degree rotation |

---

## Source Code Organization & Style
To maintain code quality and readability, all contributing developers **must** adhere to our team style guide.
**[READ THE FULL STYLE GUIDE HERE](./STYLE_GUIDE.md)**

## Git Workflow
To maintain a clean working repository, all contributing developers **must** adhere to the team contribution guide.
**[READ THE FULL CONTRIBUTION GUIDE HERE](./CONTRIBUTION_GUIDE.md)**


## Controls Map 

**Driver** XBox 360
---
| Input | Functionality |
| :--- | :--- |
| **A Button** | |
| **B Button** | |
| **X Button** | |
| **Y Button** | |
| **Left Bumper (LB)** | |
| **Right Bumper (RB)** | |
| **Left Trigger (LT)** | |
| **Right Trigger (RT)** | |
| **Back Button** | |
| **Start Button** | |
| **Left Stick Click (L3)**| |
| **Right Stick Click (R3)**| |
| **Left Stick (X/Y)** | |
| **Right Stick (X/Y)** | |
| **D-Pad (Up/Down)** | |
| **D-Pad (Left/Right)**| |

**Operator** XBox 360
---
| Input | Functionality |
| :--- | :--- |
| **A Button** | |
| **B Button** | |
| **X Button** | |
| **Y Button** | |
| **Left Bumper (LB)** | |
| **Right Bumper (RB)** | |
| **Left Trigger (LT)** | |
| **Right Trigger (RT)** | |
| **Back Button** | |
| **Start Button** | |
| **Left Stick Click (L3)**| |
| **Right Stick Click (R3)**| |
| **Left Stick (X/Y)** | |
| **Right Stick (X/Y)** | |
| **D-Pad (Up/Down)** | |
| **D-Pad (Left/Right)**| |

## Setup & Installation for Windows Machines
1. Download the most recent version of the [WPILib software](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).
2. Find the .iso in your downloads folder.
3. Use GitBash or Powershell to cofirm the authenticity of the download.
* GitBash
```bash 
shasum -a 256 ./path/to/downloaded_file.iso
```
* Powershell
```powershell 
Get-FileHash ./path/to/downloaded_file.iso -Algorithm SHA256
```

4. Navigate to the mounted drive and run the installer medium. 
5. Launch and verify isntallation.
6. Setup your working environmet. 
7. Clone this public repository for team code: 
   ```bash
   git clone https://github.com/LISD-TECHnicians/2026-REBUILT.git
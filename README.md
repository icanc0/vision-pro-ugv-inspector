# Monorepo for vision pro UGV control

## Hardware Overview

- Apple Vision Pro
- Clearpath Jackal
- Custom Computer Internals 
- LadyBug 5 360 Camera
- Livox Avia Lidar
- IQR Pan-tilt
- Kurokesu L086 motorized zoom lens
- Kurokesu USB camera C3_4K

<sub><sup><sub><sup><sub>man i can buy a whole tesla with this budget 💀</sup></sub></sup></sub></sup>

## Architecture

```mermaid
graph TD
    A[Jackal running Proxmox] --> B[Ubuntu 20.04 VM]
    A --> C[Windows 10 VM]
    
    B --> |Controls| D[LIDAR]
    D --> |Provides Data| B
    B --> |Controls| E[Jackal Motors]
    B --> |Controls| F[Zoom Camera]
    B --> |Controls| G[Pan-Tilt Platform]
    B --> |Runs| H[Calibration Software]
    D --> |Provides Data| H
    B --> |Runs| I[HTTP to ROS Image Node]
    
    C --> |Controls| J[360 Camera]
    J --> |Provides Data| H
    J --> |Saves as file| K[HTTP Image Endpoint from File]

    L[Vision Pro] --> |Commands| B
    L <-.-> |Requests 360 Images| C
    B -.-> |Zoomed Images| L
    
    K -.-> |Images| I
    I -.-> |ROS Images| H
```

**Note: VMs are graphically accelerated through SR-IOV**

## Getting started

### Vision pro
Build and run via Xcode
### Ladybug 5
Open `vision-pro-ugv-inspector\windows\ladybug\src`
Build and run on windows via VS



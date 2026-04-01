# VBDrive: integrated BLDC FOC controller

## Repository Structure

- `Core` - STM32CubeMX generated code. Nearly no user code (except for heap guard in `sysmem.c` : `_sbrk`)
- `App` - main code
- `Drivers/libcxxcanard` - cyphal over fdcan communication lib
- `Drivers/libvoltbro` - our shared library of hardware drivers. Main FOC control code lives here:
  - `Drivers/libvoltbro/voltbro/motors/bldc/bldc.[h,cpp]`: base class
  - `Drivers/libvoltbro/voltbro/motors/bldc/foc/foc.[hpp,cpp]`: foc generic class
  - `Drivers/libvoltbro/voltbro/motors/bldc/vbdrive/vbdrive.hpp`: specific class to handle this specific hardware

## Rules

- Do not add helper functions if they are used only once

## Dev notes

- Heap is disabled after initial setup
- All `std::string` should be less then 16 chars long to exploit SSO
- No new cyphal subscriptions should be created unless explicitly specified

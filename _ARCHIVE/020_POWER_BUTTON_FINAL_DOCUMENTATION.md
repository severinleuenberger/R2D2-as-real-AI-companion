# ARCHIVED: R2D2 Power Button System Documentation

**Archive Date:** January 5, 2026  
**Reason:** Content integrated into core documentation  
**Status:** This standalone document has been superseded

---

## Document Status

This document has been **archived** and is no longer the primary reference for power button system documentation. All content has been integrated into the appropriate core documentation files for better organization and discoverability.

---

## Where to Find This Information Now

The content from this document has been distributed across multiple documentation files based on the type of information:

### Hardware Details

**See:** [`002_HARDWARE_REFERENCE.md`](../002_HARDWARE_REFERENCE.md)

- **Section 2.4 (GPIO Connections):** Detailed shutdown button specifications
  - Complete GPIO wiring (Pin 32 + Pin 39)
  - Electrical specifications (pull-up, logic levels, debouncing)
  - Button hardware specifications
  - Testing procedures

- **Section 2.5 (Automation Header J42):** Detailed boot/wake button specifications
  - J42 header complete pinout
  - Pin 2 vs Pin 4 distinction (CRITICAL)
  - Boot/wake button wiring
  - Hardware-level operation

- **Section 3.8 (Control Inputs):** Complete power management overview
  - Button 1 & Button 2 specifications
  - Operational behavior
  - Testing results
  - Power control system summary table

### Service Management

**See:** [`005_SYSTEMD_SERVICES_REFERENCE.md`](../005_SYSTEMD_SERVICES_REFERENCE.md)

- **Section 10 (r2d2-powerbutton.service):** Complete service documentation
  - Service configuration and file structure
  - Code architecture (PowerButtonHandler class)
  - Logging configuration (journal + persistent log)
  - Service management commands
  - Installation and deployment procedures
  - Comprehensive troubleshooting guide
  - Testing and verification procedures

### User Guidance

**See:** [`000_UX_AND_FUNCTIONS.md`](../000_UX_AND_FUNCTIONS.md)

- **Section 5 (Physical Controls):** User-facing power button usage
  - When to use shutdown button
  - Expected behavior and timing
  - Daily usage patterns
  - Visual/audio feedback
  - User troubleshooting guide
  - Shutdown vs. leave running guidance

### System Architecture

**See:** [`001_ARCHITECTURE_OVERVIEW.md`](../001_ARCHITECTURE_OVERVIEW.md)

- **Section 7.1 (Power Button System):** System integration and architecture
  - Power control data flow
  - Software stack overview
  - Integration with other R2D2 systems
  - State machine diagram
  - Cross-references to all related documentation

---

## Original Document Content

The original content of this document (dated December 9, 2025) included:

- **Overview:** System description and status
- **Hardware Setup:** Button 1 & Button 2 wiring details
- **Software Implementation:** File locations and service configuration
- **Service Configuration:** Management commands and logging
- **Button Functions:** Detailed operation descriptions
- **Testing Results:** December 9, 2025 test verification
- **Installation:** Setup and verification procedures
- **Code Architecture:** PowerButtonHandler class description
- **Troubleshooting:** Common issues and solutions
- **Key Specifications:** Parameter table

**All of this content has been preserved and integrated** into the appropriate sections of the core documentation listed above.

---

## Why This Change Was Made

**Benefits of Integration:**

1. **Better Organization**
   - Hardware specs are with all other hardware specs (002_)
   - Service management is with all other services (005_)
   - User guidance is in the UX document (000_)
   - System integration is in architecture overview (001_)

2. **Improved Discoverability**
   - Users looking for GPIO pinouts check hardware reference
   - Users troubleshooting services check service reference
   - New users reading UX guide learn about power buttons naturally

3. **Easier Maintenance**
   - No need to update multiple documents when hardware/service changes
   - Changes to service templates automatically include power button
   - Consistency with how other subsystems are documented

4. **Follows Established Patterns**
   - Camera: Hardware in 002, no standalone doc
   - LED: Hardware in 002 + wiring guide, service in 005
   - Audio: Hardware in 002, service in 005, UX in 000
   - Power button now follows same pattern

---

## Accessing Archived Content

If you need to reference the original standalone document for historical purposes:

**Git History:**
```bash
# View this file's history
git log --follow -- 020_POWER_BUTTON_FINAL_DOCUMENTATION.md

# View original content before archiving
git show HEAD~1:020_POWER_BUTTON_FINAL_DOCUMENTATION.md
```

**This Archive File:**
- Located in `_ARCHIVE/` directory
- Preserved for reference
- Not maintained going forward
- Use integrated documentation for current information

---

## Quick Reference Links

**Need power button information? Start here:**

| Question | Document | Section |
|----------|----------|---------|
| How do I wire the buttons? | [002_HARDWARE_REFERENCE.md](../002_HARDWARE_REFERENCE.md) | 2.4, 2.5, 3.8 |
| How do I manage the service? | [005_SYSTEMD_SERVICES_REFERENCE.md](../005_SYSTEMD_SERVICES_REFERENCE.md) | 10 |
| How do I use the buttons? | [000_UX_AND_FUNCTIONS.md](../000_UX_AND_FUNCTIONS.md) | 5 |
| How does it integrate with the system? | [001_ARCHITECTURE_OVERVIEW.md](../001_ARCHITECTURE_OVERVIEW.md) | 7.1 |
| Where are the log files? | [005_SYSTEMD_SERVICES_REFERENCE.md](../005_SYSTEMD_SERVICES_REFERENCE.md) | 10 (Logging Configuration) |
| Troubleshooting shutdown button | [005_SYSTEMD_SERVICES_REFERENCE.md](../005_SYSTEMD_SERVICES_REFERENCE.md) | 10 (Troubleshooting) |
| Troubleshooting wake/boot button | [002_HARDWARE_REFERENCE.md](../002_HARDWARE_REFERENCE.md) | 3.8 |

---

**Last Updated:** January 5, 2026  
**Archive Maintainer:** R2D2 Project Team  
**For questions:** Refer to current documentation in links above


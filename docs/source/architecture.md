# Architecture

Internal architecture and design of HID ROS 2.

## System Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                      YAML Schema                              │
│                 (Single Source of Truth)                      │
└─────────────────────────┬─────────────────────────────────────┘
                          │
          ┌───────────────┴────────────────┐
          ▼                                ▼
┌──────────────────────┐         ┌──────────────────────┐
│  Code Generator      │         │  Firmware Generator  │
│  (Python)            │         │  (Python)            │
├──────────────────────┤         ├──────────────────────┤
│ • URDF Generator     │         │ • HID Descriptor     │
│ • Config Generator   │         │ • Report Structs     │
│ • Launch Generator   │         │ • Type Definitions   │
└──────────┬───────────┘         └───────────┬──────────┘
           │                                 │
           ▼                                 ▼
┌──────────────────────┐         ┌──────────────────────┐
│  ROS 2 Control       │   USB   │  Microcontroller     │
│                      │◄───────►│  Firmware            │
│ • Hardware Interface │   HID   │                      │
│ • Controllers        │         │ • HID Reports        │
│ • State Interfaces   │         │ • Application Logic  │
└──────────────────────┘         └──────────────────────┘
```

## Component Details

This section provides technical details about the internal architecture. For user-level documentation, see other guides.

## Code Organization

The repository is organized into multiple ROS 2 packages, each with a specific responsibility.

For complete architectural documentation, see ARCHITECTURE.md in the repository root.

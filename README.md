# DCEMLID v1.00

ODI MLID Driver for Direct Cable Connection between DOS computers.

## Overview

DCEMLID is a Terminate-and-Stay-Resident (TSR) network driver implementing the Novell ODI (Open Data-Link Interface) MLID specification. It enables point-to-point IPX networking between two DOS computers using:

- **Serial null-modem cables** (9600-115200 baud)
- **Parallel cables** (4-bit SPP or 8-bit EPP/PS2 mode)

The driver integrates with Novell's Link Support Layer (LSL) to provide transparent network connectivity for IPX-based applications like NetWare, games (Doom, Duke Nukem 3D), and file transfer utilities.

## Features

- Configurable MAC addresses
- Hardware (RTS/CTS) and software (XON/XOFF) flow control
- Automatic 16550 UART detection with FIFO support
- EPP and PS/2 bidirectional parallel port support
- CRC16-CCITT error detection with automatic retry
- 512-byte MTU optimized for low latency
- Automatic UMB (Upper Memory Block) loading on DOS 5.0+
- Clean unload capability

## System Requirements

### Minimum
- IBM PC/XT or compatible (8086 CPU or higher)
- DOS 3.3 or higher (DOS 5.0+ recommended for UMB support)
- 32KB free conventional memory (or 8KB with UMB)
- Novell LSL.COM

### Serial Mode
- 8250 or 16550 UART (16550 recommended for >38400 baud)
- Available COM port (COM1-COM4)
- Null-modem cable

### Parallel Mode
- Standard parallel port (LPT1-LPT3)
- PAR4: Any parallel port
- PAR8: EPP-capable or PS/2 bidirectional port

## Quick Start

### 1. Create NET.CFG

```
Link Driver DCEMLID
   PORT 3F8
   INT 4
   MODE SERIAL
   BAUD 115200
   FLOW RTS
   NODE ADDRESS 02608C123456
```

### 2. Load Drivers

```bat
LSL
DCEMLID
IPXODI
```

### 3. Verify Installation

```bat
IPXODI S
```

## Configuration Options

Configuration is specified in `NET.CFG` under a `Link Driver DCEMLID` section.

| Keyword | Values | Description |
|---------|--------|-------------|
| `PORT` | `3F8`, `2F8`, `3E8`, `2E8` (serial) / `378`, `278`, `3BC` (parallel) | I/O port base address (hex) |
| `INT` | `3`, `4`, `7` | IRQ number |
| `MODE` | `SERIAL`, `PAR4`, `PAR8` | Connection mode |
| `BAUD` | `9600`, `19200`, `38400`, `57600`, `115200` | Serial baud rate |
| `FLOW` | `RTS`, `XON` | Flow control (serial only) |
| `NODE ADDRESS` | 12 hex digits | MAC address (e.g., `02608C123456`) |

## Command Line Options

```
DCEMLID [board_number] [/?] [/U]

board_number  - Board instance (0-9), must match NET.CFG
/?            - Display help with cable wiring diagrams
/U            - Unload driver from memory
```

## Cable Wiring

### Serial Null-Modem (DB9, Hardware Flow Control)

```
Computer A          Computer B
Pin 2 (RXD) <------- Pin 3 (TXD)
Pin 3 (TXD) -------> Pin 2 (RXD)
Pin 7 (RTS) <------- Pin 8 (CTS)
Pin 8 (CTS) -------> Pin 7 (RTS)
Pin 5 (GND) <------> Pin 5 (GND)
```

### Serial Null-Modem (DB9, Software Flow Control - 3 wire)

```
Computer A          Computer B
Pin 2 (RXD) <------- Pin 3 (TXD)
Pin 3 (TXD) -------> Pin 2 (RXD)
Pin 5 (GND) <------> Pin 5 (GND)
```

### Parallel (PAR4 Mode)

Use any standard bidirectional parallel printer cable.

## Usage Examples

### Multiplayer Gaming (Doom/Duke Nukem 3D)

**Computer A NET.CFG:**
```
Link Driver DCEMLID
   PORT 3F8
   INT 4
   MODE SERIAL
   BAUD 115200
```

**Both computers:**
```bat
LSL
DCEMLID
IPXODI
DOOM -nodes 2 -port 0x869
```

### NetWare Client Connection

```bat
LSL
DCEMLID
IPXODI
NETX
F:
LOGIN
```

### Unloading (reverse order)

```bat
NETX /U
IPXODI /U
DCEMLID /U
LSL /U
```

## Building from Source

### DOS/Windows with Borland TASM

```bat
TASM /m2 /jJUMPS DCEMLID.ASM
TLINK /t DCEMLID.OBJ
```

Or using Make:
```bat
MAKE
```

### Linux with DOSBox

```bash
make dosbox
```

See [BUILD.TXT](BUILD.TXT) for detailed build instructions.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "ERROR: LSL not loaded" | Load LSL.COM before DCEMLID |
| "ERROR: No configuration in NET.CFG" | Create NET.CFG with `Link Driver DCEMLID` section |
| "WARNING: 8250 UART at high baud rate" | Reduce baud to 38400 or upgrade to 16550 |
| Slow/no transfer | Check cable wiring, ensure matching settings on both PCs |
| CRC errors | Lower baud rate, check cable quality (<10 feet) |
| Parallel not working | Set port to bidirectional/EPP in BIOS, try PAR4 mode |

## Files

| File | Description |
|------|-------------|
| `DCEMLID.COM` | Compiled driver |
| `DCEMLID.ASM` | Assembly source code |
| `DCEMLID.TXT` | Full documentation |
| `BUILD.TXT` | Build instructions |
| `NET.CFG` | Example configuration |
| `LSL.COM` | Novell Link Support Layer |
| `IPXODI.COM` | IPX protocol stack |

## Documentation

For complete documentation including all configuration options, usage examples, cable wiring diagrams, and troubleshooting guides, see [DCEMLID.TXT](DCEMLID.TXT).

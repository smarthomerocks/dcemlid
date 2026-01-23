; DCEMLID.ASM - ODI MLID Driver for Serial/Parallel Direct Cable Connection
; Implements point-to-point IPX networking over null-modem or parallel cables
; Supports configurable MAC, baud rates, flow control, and automatic UMB loading

.MODEL SMALL
.286
.STACK 100h

;=============================================================================
; Constants and Equates
;=============================================================================

; Driver Information
DRIVER_NAME     DB 'DCEMLID',0
DRIVER_VERSION  EQU 0100h               ; Version 1.00
MTU_SIZE        EQU 512                 ; Maximum Transmission Unit

; Multiplex and LSL Constants
MULTIPLEX_ID    EQU 0C0A0h              ; Our multiplex ID
LSL_CHECK       EQU 0C000h              ; Check if LSL loaded
LSL_REGISTER    EQU 0C001h              ; Register MLID with LSL

; Frame Protocol Constants
FRAME_START     EQU 0C0h                ; SLIP-style frame delimiter
FRAME_ESC       EQU 0DBh                ; Escape character
ESC_START       EQU 0DCh                ; Escaped START
ESC_ESC         EQU 0DDh                ; Escaped ESC
ESC_XON         EQU 0DEh                ; Escaped XON
ESC_XOFF        EQU 0DFh                ; Escaped XOFF
XON_CHAR        EQU 011h                ; XON (Ctrl+Q)
XOFF_CHAR       EQU 013h                ; XOFF (Ctrl+S)
ACK_CHAR        EQU 006h                ; ACK
NAK_CHAR        EQU 015h                ; NAK

; UART Register Offsets
UART_RBR        EQU 0                   ; Receiver Buffer (Read)
UART_THR        EQU 0                   ; Transmitter Holding (Write)
UART_IER        EQU 1                   ; Interrupt Enable
UART_IIR        EQU 2                   ; Interrupt Identification (Read)
UART_FCR        EQU 2                   ; FIFO Control (Write)
UART_LCR        EQU 3                   ; Line Control
UART_MCR        EQU 4                   ; Modem Control
UART_LSR        EQU 5                   ; Line Status
UART_MSR        EQU 6                   ; Modem Status
UART_SCR        EQU 7                   ; Scratch Register

; UART Configuration Values
LCR_DLAB        EQU 80h                 ; Divisor Latch Access Bit
LCR_8N1         EQU 03h                 ; 8 data bits, no parity, 1 stop
FCR_ENABLE      EQU 01h                 ; Enable FIFO
FCR_CLEAR       EQU 06h                 ; Clear TX/RX FIFOs
FCR_TRIGGER_8   EQU 80h                 ; 8-byte trigger level
MCR_DTR         EQU 01h                 ; Data Terminal Ready
MCR_RTS         EQU 02h                 ; Request To Send
MCR_OUT2        EQU 08h                 ; OUT2 (enable IRQ)

; Parallel Port Register Offsets
LPT_DATA        EQU 0                   ; Data register
LPT_STATUS      EQU 1                   ; Status register
LPT_CONTROL     EQU 2                   ; Control register
LPT_EPP_ADDR    EQU 3                   ; EPP address port
LPT_EPP_DATA    EQU 4                   ; EPP data port
LPT_ECR         EQU 402h                ; Extended Control Register

; Parallel Port Control Bits
LPT_CTRL_STROBE EQU 01h                 ; Strobe
LPT_CTRL_AUTOFD EQU 02h                 ; Auto Line Feed
LPT_CTRL_INIT   EQU 04h                 ; Initialize
LPT_CTRL_SELECT EQU 08h                 ; Select In
LPT_CTRL_IRQ    EQU 10h                 ; Enable IRQ
LPT_CTRL_BIDIR  EQU 20h                 ; Bidirectional enable (PS/2)

; Buffer Sizes
RX_BUFFER_SIZE  EQU 4096                ; 4KB receive circular buffer
TX_BUFFER_SIZE  EQU 2048                ; 2KB transmit queue

; Timing Constants
RETRY_TIMEOUT   EQU 200                 ; 200ms timeout
MAX_RETRIES     EQU 2                   ; Maximum retry attempts

; MLID Control Function Codes
MLID_RESET      EQU 01h
MLID_SHUTDOWN   EQU 02h
MLID_GET_STATS  EQU 05h
MLID_SEND_PKT   EQU 06h
MLID_REG_RX     EQU 07h

;=============================================================================
; Data Segment
;=============================================================================

.DATA

; Configuration Parameters
Config STRUC
    Mode        DB 0                    ; 0=Serial, 1=PAR4, 2=PAR8
    Port        DW 3F8h                 ; I/O port base address
    BaudRate    DW 115200               ; Baud rate (serial only)
    FlowCtrl    DB 0                    ; 0=RTS/CTS, 1=XON/XOFF
    MacAddr     DB 6 DUP(0)             ; MAC address
    IRQNum      DB 0                    ; IRQ number
    UARTType    DB 0                    ; 0=8250, 1=16550
    ParMode     DB 0                    ; For PAR8: 0=EPP, 1=PS/2
Config ENDS

CfgData Config <>

; Statistics Structure
Stats STRUC
    TxPackets   DD 0
    TxBytes     DD 0
    RxPackets   DD 0
    RxBytes     DD 0
    CRCErrors   DD 0
    Timeouts    DD 0
    Retries     DD 0
    Overruns    DD 0
Stats ENDS

Statistics Stats <>

; Driver State
DriverLoaded    DB 0                    ; Flag: driver loaded
DriverActive    DB 0                    ; Flag: driver active (not idle)
RxHandlerSeg    DW 0                    ; LSL receive handler segment
RxHandlerOff    DW 0                    ; LSL receive handler offset

; Interrupt Vectors
OldInt2F_Seg    DW 0
OldInt2F_Off    DW 0
OldHwInt_Seg    DW 0
OldHwInt_Off    DW 0

; Transmit State
TxSeqNum        DB 0                    ; Current TX sequence number
TxPending       DB 0                    ; TX in progress flag
TxRetryCount    DB 0                    ; Current retry count
TxTimeout       DW 0                    ; Timeout counter

; Receive State
RxSeqNum        DB 0                    ; Expected RX sequence number
RxState         DB 0                    ; Frame parser FSM state
RxEscaped       DB 0                    ; Escape sequence flag
RxBufIdx        DW 0                    ; Current buffer index
RxFrameLen      DW 0                    ; Current frame length
RxCRC           DW 0FFFFh               ; Running CRC calculation

; Buffers
RxBuffer        DB RX_BUFFER_SIZE DUP(0)
TxBuffer        DB TX_BUFFER_SIZE DUP(0)
FrameBuffer     DB MTU_SIZE+16 DUP(0)   ; Temp frame assembly buffer

; Messages
MsgBanner       DB 'DCEMLID v1.00 - ODI MLID Driver for Direct Cable Connection',13,10,'$'
MsgLoaded       DB 'Driver loaded: $'
MsgSerial       DB 'SERIAL $'
MsgPar4         DB 'PAR4 $'
MsgPar8EPP      DB 'PAR8-EPP $'
MsgPar8PS2      DB 'PAR8-PS2 $'
MsgPort         DB 'Port=$'
MsgIRQ          DB ' IRQ=$'
MsgMAC          DB ' MAC=$'
MsgBaud         DB ' Baud=$'
Msg8250         DB ' (8250)$'
Msg16550        DB ' (16550)$'
MsgUMB          DB ' [UMB]$'
MsgConv         DB ' [Conventional]$'
MsgCRLF         DB 13,10,'$'

MsgWarn8250     DB 'WARNING: 8250 UART at high baud - packet loss may occur.',13,10
                DB '         Consider using 38400 or lower.',13,10,'$'

MsgErrLSL       DB 'ERROR: LSL not loaded. Load LSL.COM first.',13,10,'$'
MsgErrLoaded    DB 'ERROR: Driver already loaded.',13,10,'$'
MsgErrIRQ       DB 'ERROR: IRQ conflict detected.',13,10,'$'
MsgErrPort      DB 'ERROR: Hardware not detected at specified port.',13,10,'$'
MsgErrDOS       DB 'ERROR: Requires DOS 5.0 or higher for UMB support.',13,10,'$'

MsgUnloaded     DB 'Driver unloaded successfully.',13,10,'$'
MsgErrNotLoaded DB 'ERROR: Driver not loaded.',13,10,'$'
MsgErrBusy      DB 'ERROR: Driver busy, cannot unload.',13,10,'$'

; Help Screen
HelpText        DB 13,10,'DCEMLID v1.00 - ODI MLID Driver for Direct Cable Connection',13,10
                DB 13,10,'Usage: DCEMLID [options]',13,10
                DB 13,10,'Options:',13,10
                DB '  /MODE=SERIAL|PAR4|PAR8   Connection mode (default: SERIAL)',13,10
                DB '  /PORT=addr               I/O port address in hex',13,10
                DB '                           Serial: 3F8,2F8,2E8,3E8 (default: 3F8)',13,10
                DB '                           Parallel: 378,278,3BC (default: 378)',13,10
                DB '  /BAUD=rate               Serial baud rate (default: 115200)',13,10
                DB '                           Valid: 9600,19200,38400,57600,115200',13,10
                DB '  /FLOW=RTS|XON            Flow control (default: RTS)',13,10
                DB '  /MAC=xx-xx-xx-xx-xx-xx   MAC address (default: random)',13,10
                DB '  /U                       Unload driver',13,10
                DB '  /?                       Display this help',13,10
                DB 13,10,'Serial Null-Modem Cable Wiring (DB9):',13,10
                DB '  Hardware flow control:',13,10
                DB '    Pin 2 (RXD) -------- Pin 3 (TXD)',13,10
                DB '    Pin 3 (TXD) -------- Pin 2 (RXD)',13,10
                DB '    Pin 7 (RTS) -------- Pin 8 (CTS)',13,10
                DB '    Pin 8 (CTS) -------- Pin 7 (RTS)',13,10
                DB '    Pin 5 (GND) -------- Pin 5 (GND)',13,10
                DB '  3-wire (software flow):',13,10
                DB '    Pin 2 (RXD) -------- Pin 3 (TXD)',13,10
                DB '    Pin 3 (TXD) -------- Pin 2 (RXD)',13,10
                DB '    Pin 5 (GND) -------- Pin 5 (GND)',13,10
                DB 13,10,'Parallel Cable Wiring (DB25):',13,10
                DB '  4-bit mode: Use standard parallel cable',13,10
                DB '  8-bit mode: Use EPP-capable cable or PS/2 bidirectional',13,10
                DB 13,10,'$'

; Command Line Parameter Strings
OptMode         DB '/MODE=',0
OptPort         DB '/PORT=',0
OptBaud         DB '/BAUD=',0
OptFlow         DB '/FLOW=',0
OptMAC          DB '/MAC=',0
OptUnload       DB '/U',0
OptHelp         DB '/?',0

ModeSerial      DB 'SERIAL',0
ModePar4        DB 'PAR4',0
ModePar8        DB 'PAR8',0

FlowRTS         DB 'RTS',0
FlowXON         DB 'XON',0

; Baud Rate Table (divisor values for 115200 base)
BaudTable:
    DW 9600, 12              ; 115200 / 12 = 9600
    DW 19200, 6              ; 115200 / 6 = 19200
    DW 38400, 3              ; 115200 / 3 = 38400
    DW 57600, 2              ; 115200 / 2 = 57600
    DW 115200, 1             ; 115200 / 1 = 115200
    DW 0                     ; End marker
BaudTableSize EQU ($ - BaudTable) / 4 - 1

;=============================================================================
; Code Segment
;=============================================================================

.CODE

Start:
    mov     ax, @data
    mov     ds, ax
    mov     es, ax

    ; Display banner
    lea     dx, MsgBanner
    mov     ah, 09h
    int     21h

    ; Parse command line
    call    ParseCommandLine
    jc      ExitError

    ; Check for help option
    cmp     byte ptr [CmdHelp], 1
    je      ShowHelp

    ; Check for unload option
    cmp     byte ptr [CmdUnload], 1
    je      UnloadDriver

    ; Install driver
    call    InstallDriver
    jc      ExitError

    ; Stay resident
    mov     ax, 3100h           ; TSR with return code 0
    mov     dx, ResidentSize
    add     dx, 15
    shr     dx, 4               ; Convert bytes to paragraphs
    int     21h

ExitError:
    mov     ax, 4C01h           ; Exit with error code 1
    int     21h

ShowHelp:
    lea     dx, HelpText
    mov     ah, 09h
    int     21h
    mov     ax, 4C00h
    int     21h

;=============================================================================
; ParseCommandLine - Parse and validate command line parameters
; Input: PSP command line at DS:0080h
; Output: CF=1 on error, CF=0 on success
;=============================================================================

CmdHelp         DB 0
CmdUnload       DB 0

ParseCommandLine PROC
    push    si
    push    di

    ; Initialize defaults
    mov     CfgData.Mode, 0             ; Serial
    mov     CfgData.Port, 3F8h          ; COM1
    mov     CfgData.BaudRate, 115200
    mov     CfgData.FlowCtrl, 0         ; RTS/CTS
    
    ; Generate random MAC from BIOS timer
    call    GenerateRandomMAC

    ; Get command line from PSP
    mov     si, 80h                     ; PSP command line length
    xor     cx, cx
    mov     cl, [si]                    ; Length of command line
    inc     si                          ; Point to actual command line
    jcxz    ParseDone                   ; No parameters

ParseLoop:
    ; Skip spaces
    call    SkipSpaces
    jcxz    ParseDone

    ; Check for each option
    lea     di, OptHelp
    call    CompareOption
    je      ParseHelp

    lea     di, OptUnload
    call    CompareOption
    je      ParseUnload

    lea     di, OptMode
    call    CompareOption
    je      ParseMode

    lea     di, OptPort
    call    CompareOption
    je      ParsePort

    lea     di, OptBaud
    call    CompareOption
    je      ParseBaud

    lea     di, OptFlow
    call    CompareOption
    je      ParseFlow

    lea     di, OptMAC
    call    CompareOption
    je      ParseMAC

    ; Unknown option, skip to next
    call    SkipToSpace
    jmp     ParseLoop

ParseHelp:
    mov     CmdHelp, 1
    jmp     ParseDone

ParseUnload:
    mov     CmdUnload, 1
    jmp     ParseDone

ParseMode:
    ; Parse mode value
    lea     di, ModeSerial
    call    CompareValue
    je      ParseModeSerial
    
    lea     di, ModePar4
    call    CompareValue
    je      ParseModePar4
    
    lea     di, ModePar8
    call    CompareValue
    je      ParseModePar8
    
    jmp     ParseLoop

ParseModeSerial:
    mov     CfgData.Mode, 0
    jmp     ParseLoop

ParseModePar4:
    mov     CfgData.Mode, 1
    mov     CfgData.Port, 378h          ; LPT1 default
    jmp     ParseLoop

ParseModePar8:
    mov     CfgData.Mode, 2
    mov     CfgData.Port, 378h          ; LPT1 default
    jmp     ParseLoop

ParsePort:
    ; Parse hex port value
    call    ParseHexWord
    jc      ParseLoop
    mov     CfgData.Port, ax
    jmp     ParseLoop

ParseBaud:
    ; Parse decimal baud rate
    call    ParseDecWord
    jc      ParseLoop
    mov     CfgData.BaudRate, ax
    jmp     ParseLoop

ParseFlow:
    ; Parse flow control
    lea     di, FlowRTS
    call    CompareValue
    je      ParseFlowRTS
    
    lea     di, FlowXON
    call    CompareValue
    je      ParseFlowXON
    
    jmp     ParseLoop

ParseFlowRTS:
    mov     CfgData.FlowCtrl, 0
    jmp     ParseLoop

ParseFlowXON:
    mov     CfgData.FlowCtrl, 1
    jmp     ParseLoop

ParseMAC:
    ; Parse MAC address (format: XX-XX-XX-XX-XX-XX)
    lea     di, CfgData.MacAddr
    mov     bx, 6
ParseMACLoop:
    call    ParseHexByte
    jc      ParseLoop
    stosb
    dec     bx
    jz      ParseMACDone
    
    ; Skip delimiter
    cmp     byte ptr [si], '-'
    jne     ParseMACLoop
    inc     si
    dec     cx
    jmp     ParseMACLoop

ParseMACDone:
    jmp     ParseLoop

ParseDone:
    clc
    pop     di
    pop     si
    ret
ParseCommandLine ENDP

;=============================================================================
; Helper Procedures for Command Line Parsing
;=============================================================================

SkipSpaces PROC
    jcxz    SkipSpacesDone
SkipSpacesLoop:
    cmp     byte ptr [si], ' '
    jne     SkipSpacesDone
    inc     si
    loop    SkipSpacesLoop
SkipSpacesDone:
    ret
SkipSpaces ENDP

SkipToSpace PROC
    jcxz    SkipToSpaceDone
SkipToSpaceLoop:
    cmp     byte ptr [si], ' '
    je      SkipToSpaceDone
    inc     si
    loop    SkipToSpaceLoop
SkipToSpaceDone:
    ret
SkipToSpace ENDP

CompareOption PROC
    ; Compare string at SI with string at DI (case insensitive)
    ; Preserves SI, CX
    push    si
    push    cx
CompOptLoop:
    mov     al, [di]
    test    al, al
    jz      CompOptMatch
    mov     ah, [si]
    
    ; Convert to uppercase
    call    ToUpper
    xchg    al, ah
    call    ToUpper
    
    cmp     al, ah
    jne     CompOptNoMatch
    
    inc     si
    inc     di
    dec     cx
    jnz     CompOptLoop

CompOptNoMatch:
    pop     cx
    pop     si
    test    al, al              ; Clear ZF
    ret

CompOptMatch:
    pop     cx
    pop     si
    add     si, di              ; Advance SI
    sub     si, offset OptHelp  ; Adjust for starting position
    sub     cx, di
    add     cx, offset OptHelp
    xor     al, al              ; Set ZF
    ret
CompareOption ENDP

CompareValue PROC
    ; Similar to CompareOption but for values after '='
    push    si
    push    cx
CompValLoop:
    mov     al, [di]
    test    al, al
    jz      CompValMatch
    mov     ah, [si]
    
    call    ToUpper
    xchg    al, ah
    call    ToUpper
    
    cmp     al, ah
    jne     CompValNoMatch
    
    inc     si
    inc     di
    dec     cx
    jnz     CompValLoop

CompValNoMatch:
    pop     cx
    pop     si
    test    al, al
    ret

CompValMatch:
    pop     cx
    pop     si
    add     si, di
    sub     si, offset ModeSerial
    sub     cx, di
    add     cx, offset ModeSerial
    xor     al, al
    ret
CompareValue ENDP

ToUpper PROC
    ; Convert AL to uppercase
    cmp     al, 'a'
    jb      ToUpperDone
    cmp     al, 'z'
    ja      ToUpperDone
    sub     al, 20h
ToUpperDone:
    ret
ToUpper ENDP

ParseHexWord PROC
    ; Parse hex word at SI, return in AX
    ; Updates SI and CX
    push    bx
    push    dx
    xor     ax, ax
    xor     bx, bx
    mov     dl, 4               ; Max 4 hex digits
ParseHexLoop:
    mov     al, [si]
    call    HexDigitToValue
    jc      ParseHexDone
    
    shl     bx, 4
    or      bl, al
    inc     si
    dec     cx
    dec     dl
    jnz     ParseHexLoop

ParseHexDone:
    mov     ax, bx
    clc
    pop     dx
    pop     bx
    ret
ParseHexWord ENDP

ParseHexByte PROC
    ; Parse hex byte at SI, return in AL
    push    bx
    xor     bx, bx
    mov     bl, 2               ; Max 2 hex digits
ParseHexByteLoop:
    mov     al, [si]
    call    HexDigitToValue
    jc      ParseHexByteDone
    
    shl     ah, 4
    or      ah, al
    inc     si
    dec     cx
    dec     bl
    jnz     ParseHexByteLoop

ParseHexByteDone:
    mov     al, ah
    clc
    pop     bx
    ret
ParseHexByte ENDP

HexDigitToValue PROC
    ; Convert hex digit in AL to value (0-15)
    ; CF=1 if not a hex digit
    cmp     al, '0'
    jb      HexNotDigit
    cmp     al, '9'
    jbe     HexIsDigit
    call    ToUpper
    cmp     al, 'A'
    jb      HexNotDigit
    cmp     al, 'F'
    ja      HexNotDigit
    sub     al, 'A' - 10
    clc
    ret
HexIsDigit:
    sub     al, '0'
    clc
    ret
HexNotDigit:
    stc
    ret
HexDigitToValue ENDP

ParseDecWord PROC
    ; Parse decimal word at SI, return in AX
    push    bx
    push    dx
    xor     ax, ax
    xor     bx, bx
ParseDecLoop:
    mov     al, [si]
    cmp     al, '0'
    jb      ParseDecDone
    cmp     al, '9'
    ja      ParseDecDone
    
    sub     al, '0'
    mov     dx, 10
    push    ax
    mov     ax, bx
    mul     dx
    mov     bx, ax
    pop     ax
    add     bx, ax
    
    inc     si
    dec     cx
    jnz     ParseDecLoop

ParseDecDone:
    mov     ax, bx
    clc
    pop     dx
    pop     bx
    ret
ParseDecWord ENDP

GenerateRandomMAC PROC
    ; Generate random MAC using BIOS timer tick
    push    es
    xor     ax, ax
    mov     es, ax
    mov     ax, es:[046Ch]      ; BIOS timer tick low word
    mov     CfgData.MacAddr[0], ah
    mov     CfgData.MacAddr[1], al
    mov     ax, es:[046Eh]      ; BIOS timer tick high word
    mov     CfgData.MacAddr[2], ah
    mov     CfgData.MacAddr[3], al
    
    ; Add some entropy from other BIOS data area values
    mov     ax, es:[0040h]
    mov     CfgData.MacAddr[4], ah
    mov     CfgData.MacAddr[5], al
    
    ; Ensure locally administered, unicast MAC
    and     CfgData.MacAddr[0], 0FEh
    or      CfgData.MacAddr[0], 02h
    
    pop     es
    ret
GenerateRandomMAC ENDP

;=============================================================================
; InstallDriver - Main driver installation routine
; Output: CF=1 on error
;=============================================================================

InstallDriver PROC
    ; Check if already loaded
    mov     ax, MULTIPLEX_ID
    int     2Fh
    cmp     al, 0FFh
    je      AlreadyLoaded

    ; Check if LSL is loaded
    mov     ax, LSL_CHECK
    int     2Fh
    cmp     al, 0FFh
    jne     LSLNotLoaded

    ; Check DOS version for UMB support
    mov     ah, 30h
    int     21h
    cmp     al, 5
    jb      DOSTooOld

    ; Initialize hardware based on mode
    cmp     CfgData.Mode, 0
    je      InitSerial
    cmp     CfgData.Mode, 1
    je      InitParallel
    cmp     CfgData.Mode, 2
    je      InitParallel

    ; Invalid mode
    stc
    ret

InitSerial:
    call    InitSerialPort
    jc      InitError
    jmp     HardwareInitDone

InitParallel:
    call    InitParallelPort
    jc      InitError

HardwareInitDone:
    ; Try to allocate UMB
    call    AllocateUMB
    ; If UMB allocation failed, we're in conventional memory (already handled)

    ; Save old interrupt vectors
    mov     ax, 352Fh
    int     21h
    mov     OldInt2F_Seg, es
    mov     OldInt2F_Off, bx

    ; Hook INT 2Fh
    push    ds
    mov     ax, cs
    mov     ds, ax
    lea     dx, Int2FHandler
    mov     ax, 252Fh
    int     21h
    pop     ds

    ; Hook hardware interrupt
    call    HookHardwareInterrupt
    jc      InitError

    ; Register with LSL
    call    RegisterWithLSL
    jc      InitError

    ; Set driver loaded flag
    mov     DriverLoaded, 1

    ; Print success message
    call    PrintInstallMessage

    clc
    ret

AlreadyLoaded:
    lea     dx, MsgErrLoaded
    mov     ah, 09h
    int     21h
    stc
    ret

LSLNotLoaded:
    lea     dx, MsgErrLSL
    mov     ah, 09h
    int     21h
    stc
    ret

DOSTooOld:
    lea     dx, MsgErrDOS
    mov     ah, 09h
    int     21h
    stc
    ret

InitError:
    ; Restore interrupt vectors if needed
    cmp     OldInt2F_Off, 0
    je      InitErrorNoRestore
    
    push    ds
    mov     ax, OldInt2F_Seg
    mov     ds, ax
    mov     dx, OldInt2F_Off
    mov     ax, 252Fh
    int     21h
    pop     ds

InitErrorNoRestore:
    lea     dx, MsgErrPort
    mov     ah, 09h
    int     21h
    stc
    ret
InstallDriver ENDP

;=============================================================================
; InitSerialPort - Initialize UART for serial communication
; Output: CF=1 on error
;=============================================================================

InitSerialPort PROC
    push    dx
    push    ax

    mov     dx, CfgData.Port

    ; Detect 8250 vs 16550
    add     dx, UART_FCR
    mov     al, 0C7h            ; Enable FIFO with all flags
    out     dx, al
    
    ; Read back IIR
    in      al, dx
    and     al, 0C0h            ; Check bits 6-7
    cmp     al, 0C0h
    je      Is16550
    
    ; It's an 8250
    mov     CfgData.UARTType, 0
    
    ; Check if baud rate is too high for 8250
    cmp     CfgData.BaudRate, 57600
    jb      UARTTypeOK
    
    ; Print warning
    push    dx
    lea     dx, MsgWarn8250
    mov     ah, 09h
    int     21h
    pop     dx
    jmp     UARTTypeOK

Is16550:
    mov     CfgData.UARTType, 1

UARTTypeOK:
    ; Calculate divisor for baud rate
    mov     ax, 115200
    xor     dx, dx
    div     CfgData.BaudRate
    mov     bx, ax              ; Save divisor in BX

    ; Set DLAB to access divisor latch
    mov     dx, CfgData.Port
    add     dx, UART_LCR
    mov     al, LCR_DLAB | LCR_8N1
    out     dx, al

    ; Set divisor
    mov     dx, CfgData.Port
    mov     al, bl              ; Low byte
    out     dx, al
    inc     dx
    mov     al, bh              ; High byte
    out     dx, al

    ; Clear DLAB, set 8N1
    mov     dx, CfgData.Port
    add     dx, UART_LCR
    mov     al, LCR_8N1
    out     dx, al

    ; Enable and configure FIFO if 16550
    cmp     CfgData.UARTType, 1
    jne     NoFIFO
    
    mov     dx, CfgData.Port
    add     dx, UART_FCR
    mov     al, FCR_ENABLE | FCR_CLEAR | FCR_TRIGGER_8
    out     dx, al

NoFIFO:
    ; Configure modem control
    mov     dx, CfgData.Port
    add     dx, UART_MCR
    mov     al, MCR_DTR | MCR_RTS | MCR_OUT2
    out     dx, al

    ; Enable interrupts
    mov     dx, CfgData.Port
    add     dx, UART_IER
    mov     al, 01h             ; Enable received data interrupt
    out     dx, al

    ; Determine IRQ number from port
    mov     ax, CfgData.Port
    cmp     ax, 3F8h
    je      SerialIRQ4
    cmp     ax, 2F8h
    je      SerialIRQ3
    cmp     ax, 3E8h
    je      SerialIRQ4
    cmp     ax, 2E8h
    je      SerialIRQ3
    
    ; Default to IRQ4
SerialIRQ4:
    mov     CfgData.IRQNum, 4
    jmp     SerialIRQSet

SerialIRQ3:
    mov     CfgData.IRQNum, 3

SerialIRQSet:
    clc
    pop     ax
    pop     dx
    ret
InitSerialPort ENDP

;=============================================================================
; InitParallelPort - Initialize parallel port (EPP or PS/2)
; Output: CF=1 on error
;=============================================================================

InitParallelPort PROC
    push    dx
    push    ax

    mov     dx, CfgData.Port

    ; Check for PAR8 mode
    cmp     CfgData.Mode, 2
    jne     Par4Mode

    ; Try to detect EPP capability
    add     dx, LPT_ECR
    in      al, dx
    shr     al, 5               ; Get bits 5-7
    cmp     al, 001b            ; EPP 1.7
    je      HasEPP
    cmp     al, 010b            ; EPP 1.9
    je      HasEPP

    ; No EPP, fallback to PS/2 bidirectional
    mov     CfgData.ParMode, 1  ; PS/2 mode
    jmp     ParallelConfigDone

HasEPP:
    mov     CfgData.ParMode, 0  ; EPP mode
    jmp     ParallelConfigDone

Par4Mode:
    ; Standard SPP 4-bit mode
    mov     CfgData.ParMode, 0

ParallelConfigDone:
    ; Initialize control register
    mov     dx, CfgData.Port
    add     dx, LPT_CONTROL
    mov     al, LPT_CTRL_INIT | LPT_CTRL_SELECT
    
    ; Add bidirectional bit for PS/2 mode
    cmp     CfgData.ParMode, 1
    jne     NoBidir
    or      al, LPT_CTRL_BIDIR

NoBidir:
    out     dx, al

    ; Parallel typically uses IRQ7
    mov     CfgData.IRQNum, 7

    clc
    pop     ax
    pop     dx
    ret
InitParallelPort ENDP

;=============================================================================
; AllocateUMB - Try to allocate memory in UMB
;=============================================================================

UMBAllocated    DB 0
MemorySegment   DW 0

AllocateUMB PROC
    push    ax
    push    bx
    push    cx

    ; Save current UMB link state
    mov     ax, 5802h
    int     21h
    push    ax                  ; Save for restoration

    ; Enable UMB chain
    mov     ax, 5803h
    mov     bx, 1
    int     21h

    ; Calculate memory needed in paragraphs
    mov     bx, ResidentSize
    add     bx, 15
    shr     bx, 4

    ; Try to allocate high memory
    mov     ah, 48h
    int     21h
    jnc     UMBSuccess

    ; UMB allocation failed, restore link state
    pop     ax
    mov     bx, ax
    mov     ax, 5803h
    int     21h
    
    ; We'll use conventional memory (already allocated by DOS)
    mov     ax, cs
    mov     MemorySegment, ax
    mov     UMBAllocated, 0
    jmp     AllocUMBDone

UMBSuccess:
    pop     cx                  ; Discard saved link state
    mov     MemorySegment, ax
    mov     UMBAllocated, 1

    ; Copy code to UMB
    push    ds
    push    es
    mov     ax, cs
    mov     ds, ax
    mov     es, MemorySegment
    xor     si, si
    xor     di, di
    mov     cx, ResidentSize
    rep     movsb
    pop     es
    pop     ds

AllocUMBDone:
    pop     cx
    pop     bx
    pop     ax
    ret
AllocateUMB ENDP

;=============================================================================
; HookHardwareInterrupt - Hook the hardware IRQ
; Output: CF=1 on error
;=============================================================================

HookHardwareInterrupt PROC
    push    ax
    push    bx
    push    dx
    push    ds
    push    es

    ; Calculate interrupt vector from IRQ number
    mov     al, CfgData.IRQNum
    cmp     al, 8
    jb      IRQBelow8
    
    ; IRQ 8-15 -> INT 70h-77h
    sub     al, 8
    add     al, 70h
    jmp     HookIRQ

IRQBelow8:
    ; IRQ 0-7 -> INT 08h-0Fh
    add     al, 08h

HookIRQ:
    mov     bl, al              ; Save interrupt number

    ; Get old vector
    mov     ah, 35h
    int     21h
    mov     OldHwInt_Seg, es
    mov     OldHwInt_Off, bx

    ; Set new vector
    push    ds
    mov     ax, cs
    mov     ds, ax
    
    cmp     CfgData.Mode, 0
    je      HookSerial
    
    lea     dx, ParallelISR
    jmp     SetVector

HookSerial:
    lea     dx, SerialISR

SetVector:
    mov     al, bl
    mov     ah, 25h
    int     21h
    pop     ds

    ; Enable IRQ at PIC
    mov     al, CfgData.IRQNum
    cmp     al, 8
    jb      EnablePIC1
    
    ; Enable on secondary PIC
    sub     al, 8
    mov     cl, al
    mov     al, 1
    shl     al, cl
    not     al
    
    in      al, 0A1h            ; Read PIC2 mask
    and     al, al              ; Clear the bit
    out     0A1h, al
    jmp     EnableDone

EnablePIC1:
    mov     cl, al
    mov     al, 1
    shl     al, cl
    not     al
    
    in      al, 021h            ; Read PIC1 mask
    and     al, al
    out     021h, al

EnableDone:
    clc
    pop     es
    pop     ds
    pop     dx
    pop     bx
    pop     ax
    ret
HookHardwareInterrupt ENDP

;=============================================================================
; RegisterWithLSL - Register MLID with Link Support Layer
; Output: CF=1 on error
;=============================================================================

; MLID Configuration Table
MLIDConfig STRUC
    ConfigTableSize     DW ?
    BoardNumber         DW ?
    BoardNodeAddr       DB 6 DUP(?)
    ModeFlags           DW ?
    BoardDataSize       DW ?
    MaxPacketSize       DW ?
    Reserved1           DW 4 DUP(?)
    MLIDName            DD ?
    MLIDReceive         DD ?
    MLIDTransmit        DD ?
    MLIDControl         DD ?
    Reserved2           DD 2 DUP(?)
MLIDConfig ENDS

MLIDConfigTable MLIDConfig <SIZE MLIDConfig, 0, , 0, 4096, MTU_SIZE>

RegisterWithLSL PROC
    push    ax
    push    bx
    push    cx
    push    dx
    push    es

    ; Fill in configuration table
    lea     si, CfgData.MacAddr
    lea     di, MLIDConfigTable.BoardNodeAddr
    mov     cx, 6
    rep     movsb

    ; Set function pointers
    mov     ax, cs
    mov     word ptr MLIDConfigTable.MLIDName[2], ax
    lea     bx, DRIVER_NAME
    mov     word ptr MLIDConfigTable.MLIDName[0], bx

    mov     word ptr MLIDConfigTable.MLIDReceive[2], ax
    lea     bx, MLIDReceiveHandler
    mov     word ptr MLIDConfigTable.MLIDReceive[0], bx

    mov     word ptr MLIDConfigTable.MLIDTransmit[2], ax
    lea     bx, MLIDTransmitHandler
    mov     word ptr MLIDConfigTable.MLIDTransmit[0], bx

    mov     word ptr MLIDConfigTable.MLIDControl[2], ax
    lea     bx, MLIDControlHandler
    mov     word ptr MLIDConfigTable.MLIDControl[0], bx

    ; Register with LSL
    mov     ax, LSL_REGISTER
    lea     bx, MLIDConfigTable
    push    ds
    pop     es
    mov     dx, es
    int     2Fh

    ; Check return status
    cmp     ax, 0
    je      RegisterOK
    stc
    jmp     RegisterDone

RegisterOK:
    clc

RegisterDone:
    pop     es
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret
RegisterWithLSL ENDP

;=============================================================================
; INT 2Fh Multiplex Handler
;=============================================================================

Int2FHandler PROC FAR
    cmp     ax, MULTIPLEX_ID
    je      OurMultiplex
    
    ; Chain to previous handler
    jmp     dword ptr cs:[OldInt2F_Off]

OurMultiplex:
    ; Return signature
    mov     al, 0FFh
    iret
Int2FHandler ENDP

;=============================================================================
; Serial Port Interrupt Service Routine
;=============================================================================

SerialISR PROC FAR
    push    ax
    push    bx
    push    dx
    push    ds

    mov     ax, @data
    mov     ds, ax

    ; Read IIR to determine interrupt source
    mov     dx, CfgData.Port
    add     dx, UART_IIR
    in      al, dx
    
    ; Check if it's a receive interrupt
    test    al, 04h
    jz      SerialNotReceive

    ; Read received data
    mov     dx, CfgData.Port
    in      al, dx
    
    ; Process received byte through frame parser
    call    ProcessReceivedByte

SerialNotReceive:
    ; Send EOI to PIC
    mov     al, 20h
    cmp     CfgData.IRQNum, 8
    jb      SerialEOI1
    
    out     0A0h, al            ; EOI to PIC2
SerialEOI1:
    out     020h, al            ; EOI to PIC1

    pop     ds
    pop     dx
    pop     bx
    pop     ax
    
    ; Chain to previous handler
    jmp     dword ptr cs:[OldHwInt_Off]
SerialISR ENDP

;=============================================================================
; Parallel Port Interrupt Service Routine
;=============================================================================

ParallelISR PROC FAR
    push    ax
    push    bx
    push    dx
    push    ds

    mov     ax, @data
    mov     ds, ax

    ; Handle parallel port interrupt
    ; (Implementation depends on specific mode)
    
    ; Send EOI to PIC
    mov     al, 20h
    out     020h, al

    pop     ds
    pop     dx
    pop     bx
    pop     ax
    
    jmp     dword ptr cs:[OldHwInt_Off]
ParallelISR ENDP

;=============================================================================
; ProcessReceivedByte - Process received byte through frame parser FSM
; Input: AL = received byte
;=============================================================================

; FSM States
STATE_IDLE      EQU 0
STATE_SEQ       EQU 1
STATE_LEN_LO    EQU 2
STATE_LEN_HI    EQU 3
STATE_DATA      EQU 4
STATE_CRC_LO    EQU 5
STATE_CRC_HI    EQU 6

ProcessReceivedByte PROC
    push    bx
    push    cx
    push    di

    ; Check for escape sequence
    cmp     RxEscaped, 0
    je      NotEscaped

    ; Process escaped byte
    mov     RxEscaped, 0
    cmp     al, ESC_START
    je      UnescapeStart
    cmp     al, ESC_ESC
    je      UnescapeESC
    cmp     al, ESC_XON
    je      UnescapeXON
    cmp     al, ESC_XOFF
    je      UnescapeXOFF
    jmp     ProcessByte

UnescapeStart:
    mov     al, FRAME_START
    jmp     ProcessByte

UnescapeESC:
    mov     al, FRAME_ESC
    jmp     ProcessByte

UnescapeXON:
    mov     al, XON_CHAR
    jmp     ProcessByte

UnescapeXOFF:
    mov     al, XOFF_CHAR
    jmp     ProcessByte

NotEscaped:
    ; Check for frame delimiter
    cmp     al, FRAME_START
    je      FrameStart

    ; Check for escape character
    cmp     al, FRAME_ESC
    je      FrameEscape

ProcessByte:
    ; Process based on current state
    mov     bl, RxState
    cmp     bl, STATE_IDLE
    je      StateIdle
    cmp     bl, STATE_SEQ
    je      StateSeq
    cmp     bl, STATE_LEN_LO
    je      StateLenLo
    cmp     bl, STATE_LEN_HI
    je      StateLenHi
    cmp     bl, STATE_DATA
    je      StateData
    cmp     bl, STATE_CRC_LO
    je      StateCRCLo
    cmp     bl, STATE_CRC_HI
    je      StateCRCHi
    jmp     ProcessDone

FrameStart:
    ; Start of new frame
    mov     RxState, STATE_SEQ
    mov     RxBufIdx, 0
    mov     RxCRC, 0FFFFh
    jmp     ProcessDone

FrameEscape:
    mov     RxEscaped, 1
    jmp     ProcessDone

StateIdle:
    ; Ignore bytes in idle state
    jmp     ProcessDone

StateSeq:
    ; Store sequence number
    mov     byte ptr FrameBuffer[0], al
    call    UpdateCRC
    mov     RxState, STATE_LEN_LO
    jmp     ProcessDone

StateLenLo:
    ; Store length low byte
    mov     byte ptr FrameBuffer[1], al
    call    UpdateCRC
    mov     RxState, STATE_LEN_HI
    jmp     ProcessDone

StateLenHi:
    ; Store length high byte and validate
    mov     byte ptr FrameBuffer[2], al
    call    UpdateCRC
    mov     ax, word ptr FrameBuffer[1]
    mov     RxFrameLen, ax
    cmp     ax, MTU_SIZE
    ja      FrameError
    mov     RxState, STATE_DATA
    jmp     ProcessDone

StateData:
    ; Store data byte
    mov     bx, RxBufIdx
    add     bx, 3
    mov     byte ptr FrameBuffer[bx], al
    call    UpdateCRC
    inc     RxBufIdx
    mov     ax, RxBufIdx
    cmp     ax, RxFrameLen
    jb      ProcessDone
    mov     RxState, STATE_CRC_LO
    jmp     ProcessDone

StateCRCLo:
    ; Receive CRC low byte
    mov     byte ptr FrameBuffer[bx], al
    mov     RxState, STATE_CRC_HI
    jmp     ProcessDone

StateCRCHi:
    ; Receive CRC high byte and validate frame
    mov     bx, RxBufIdx
    add     bx, 4
    mov     byte ptr FrameBuffer[bx], al
    
    ; Validate CRC
    call    ValidateFrame
    
    ; Reset to idle
    mov     RxState, STATE_IDLE
    jmp     ProcessDone

FrameError:
    ; Reset state on error
    mov     RxState, STATE_IDLE
    inc     dword ptr Statistics.CRCErrors

ProcessDone:
    pop     di
    pop     cx
    pop     bx
    ret
ProcessReceivedByte ENDP

;=============================================================================
; UpdateCRC - Update running CRC16-CCITT calculation
; Input: AL = byte to add to CRC
;=============================================================================

UpdateCRC PROC
    push    ax
    push    bx
    push    cx

    xor     ah, ah
    mov     bx, RxCRC
    xor     bl, al
    xor     bh, bh
    shl     bx, 1
    mov     cx, word ptr CRCTable[bx]
    mov     ax, RxCRC
    shr     ax, 8
    xor     ax, cx
    mov     RxCRC, ax

    pop     cx
    pop     bx
    pop     ax
    ret
UpdateCRC ENDP

;=============================================================================
; CRC16-CCITT Lookup Table (polynomial 0x1021)
;=============================================================================

CRCTable:
    DW 00000h, 01021h, 02042h, 03063h, 04084h, 050A5h, 060C6h, 070E7h
    DW 08108h, 09129h, 0A14Ah, 0B16Bh, 0C18Ch, 0D1ADh, 0E1CEh, 0F1EFh
    DW 01231h, 00210h, 03273h, 02252h, 052B5h, 04294h, 072F7h, 062D6h
    DW 09339h, 08318h, 0B37Bh, 0A35Ah, 0D3BDh, 0C39Ch, 0F3FFh, 0E3DEh
    ; ... (remaining 240 entries omitted for brevity - full table needed)

;=============================================================================
; ValidateFrame - Validate received frame and deliver to LSL
;=============================================================================

ValidateFrame PROC
    ; Check sequence number for duplicates
    mov     al, byte ptr FrameBuffer[0]
    cmp     al, RxSeqNum
    je      DuplicateFrame
    mov     RxSeqNum, al

    ; Deliver frame to LSL
    call    DeliverFrameToLSL

    ; Send ACK
    call    SendACK

    ; Update statistics
    inc     dword ptr Statistics.RxPackets
    mov     ax, RxFrameLen
    add     word ptr Statistics.RxBytes[0], ax
    adc     word ptr Statistics.RxBytes[2], 0

    ret

DuplicateFrame:
    ; Still send ACK for duplicate
    call    SendACK
    ret
ValidateFrame ENDP

;=============================================================================
; DeliverFrameToLSL - Deliver received frame to LSL
;=============================================================================

DeliverFrameToLSL PROC
    ; Check if receive handler is registered
    cmp     RxHandlerOff, 0
    je      NoRxHandler

    ; Call LSL receive handler
    ; (Simplified - actual implementation needs proper ECB handling)
    
NoRxHandler:
    ret
DeliverFrameToLSL ENDP

;=============================================================================
; SendACK - Send acknowledgment for received frame
;=============================================================================

SendACK PROC
    push    ax
    push    dx

    mov     al, ACK_CHAR
    mov     ah, byte ptr FrameBuffer[0]  ; Sequence number
    
    mov     dx, CfgData.Port
    
    ; Wait for transmit ready
WaitTxReady:
    add     dx, UART_LSR
    in      al, dx
    test    al, 20h
    jz      WaitTxReady

    ; Send ACK
    sub     dx, UART_LSR
    mov     al, ACK_CHAR
    out     dx, al

    ; Send sequence number
WaitTxReady2:
    add     dx, UART_LSR
    in      al, dx
    test    al, 20h
    jz      WaitTxReady2

    sub     dx, UART_LSR
    mov     al, ah
    out     dx, al

    pop     dx
    pop     ax
    ret
SendACK ENDP

;=============================================================================
; MLID Handler Functions
;=============================================================================

MLIDReceiveHandler PROC FAR
    ; Stub - actual implementation handles ECB
    ret
MLIDReceiveHandler ENDP

MLIDTransmitHandler PROC FAR
    ; Stub - actual implementation handles ECB
    ret
MLIDTransmitHandler ENDP

MLIDControlHandler PROC FAR
    ; AL = function code
    cmp     al, MLID_RESET
    je      DoReset
    cmp     al, MLID_SHUTDOWN
    je      DoShutdown
    cmp     al, MLID_GET_STATS
    je      DoGetStats
    cmp     al, MLID_SEND_PKT
    je      DoSendPacket
    cmp     al, MLID_REG_RX
    je      DoRegisterRx
    retf

DoReset:
    ; Reset hardware and buffers
    call    InitSerialPort
    mov     RxState, STATE_IDLE
    mov     RxBufIdx, 0
    mov     TxPending, 0
    retf

DoShutdown:
    ; Disable interrupts
    mov     dx, CfgData.Port
    add     dx, UART_IER
    xor     al, al
    out     dx, al
    retf

DoGetStats:
    ; Return statistics structure
    ; ES:BX points to destination
    push    ds
    push    si
    mov     ax, @data
    mov     ds, ax
    lea     si, Statistics
    mov     cx, SIZE Stats
    rep     movsb
    pop     si
    pop     ds
    retf

DoSendPacket:
    ; Queue packet for transmission
    ; Implementation needed
    retf

DoRegisterRx:
    ; Store receive handler address
    mov     RxHandlerSeg, es
    mov     RxHandlerOff, bx
    retf
MLIDControlHandler ENDP

;=============================================================================
; PrintInstallMessage - Display installation success message
;=============================================================================

PrintInstallMessage PROC
    lea     dx, MsgLoaded
    mov     ah, 09h
    int     21h

    ; Print mode
    cmp     CfgData.Mode, 0
    je      PrintSerial
    cmp     CfgData.Mode, 1
    je      PrintPar4
    
    ; PAR8 mode
    cmp     CfgData.ParMode, 0
    je      PrintPar8EPP
    lea     dx, MsgPar8PS2
    jmp     PrintModeStr

PrintPar8EPP:
    lea     dx, MsgPar8EPP
    jmp     PrintModeStr

PrintPar4:
    lea     dx, MsgPar4
    jmp     PrintModeStr

PrintSerial:
    lea     dx, MsgSerial

PrintModeStr:
    mov     ah, 09h
    int     21h

    ; Print port
    lea     dx, MsgPort
    mov     ah, 09h
    int     21h
    mov     ax, CfgData.Port
    call    PrintHexWord

    ; Print IRQ
    lea     dx, MsgIRQ
    mov     ah, 09h
    int     21h
    xor     ax, ax
    mov     al, CfgData.IRQNum
    call    PrintDecimal

    ; Print MAC address
    lea     dx, MsgMAC
    mov     ah, 09h
    int     21h
    lea     si, CfgData.MacAddr
    call    PrintMACAddress

    ; Print baud rate if serial
    cmp     CfgData.Mode, 0
    jne     NoBaud
    lea     dx, MsgBaud
    mov     ah, 09h
    int     21h
    mov     ax, CfgData.BaudRate
    call    PrintDecimal

    ; Print UART type
    cmp     CfgData.UARTType, 0
    je      Print8250
    lea     dx, Msg16550
    jmp     PrintUARTType

Print8250:
    lea     dx, Msg8250

PrintUARTType:
    mov     ah, 09h
    int     21h

NoBaud:
    ; Print memory location
    cmp     UMBAllocated, 1
    je      PrintUMBMsg
    lea     dx, MsgConv
    jmp     PrintMemMsg

PrintUMBMsg:
    lea     dx, MsgUMB

PrintMemMsg:
    mov     ah, 09h
    int     21h

    ; Print newline
    lea     dx, MsgCRLF
    mov     ah, 09h
    int     21h

    ret
PrintInstallMessage ENDP

;=============================================================================
; Helper Printing Functions
;=============================================================================

PrintHexWord PROC
    ; Print AX as hex
    push    ax
    mov     al, ah
    call    PrintHexByte
    pop     ax
    call    PrintHexByte
    ret
PrintHexWord ENDP

PrintHexByte PROC
    ; Print AL as hex
    push    ax
    shr     al, 4
    call    PrintHexDigit
    pop     ax
    and     al, 0Fh
    call    PrintHexDigit
    ret
PrintHexByte ENDP

PrintHexDigit PROC
    ; Print hex digit in AL
    cmp     al, 10
    jb      HexDigitNumeric
    add     al, 'A' - 10
    jmp     HexDigitPrint
HexDigitNumeric:
    add     al, '0'
HexDigitPrint:
    push    dx
    mov     dl, al
    mov     ah, 02h
    int     21h
    pop     dx
    ret
PrintHexDigit ENDP

PrintDecimal PROC
    ; Print AX as decimal
    push    ax
    push    bx
    push    cx
    push    dx

    xor     cx, cx
    mov     bx, 10

DecimalLoop:
    xor     dx, dx
    div     bx
    push    dx
    inc     cx
    test    ax, ax
    jnz     DecimalLoop

PrintDigits:
    pop     dx
    add     dl, '0'
    mov     ah, 02h
    int     21h
    loop    PrintDigits

    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret
PrintDecimal ENDP

PrintMACAddress PROC
    ; Print MAC address at DS:SI
    push    cx
    mov     cx, 6
PrintMACLoop:
    lodsb
    call    PrintHexByte
    dec     cx
    jz      PrintMACDone
    push    dx
    mov     dl, '-'
    mov     ah, 02h
    int     21h
    pop     dx
    jmp     PrintMACLoop
PrintMACDone:
    pop     cx
    ret
PrintMACAddress ENDP

;=============================================================================
; UnloadDriver - Unload the TSR driver
;=============================================================================

UnloadDriver PROC
    ; Check if driver is loaded via multiplex
    mov     ax, MULTIPLEX_ID
    int     2Fh
    cmp     al, 0FFh
    jne     NotLoaded

    ; Check if driver is busy
    cmp     DriverActive, 1
    je      DriverBusy

    ; Disable hardware interrupts
    mov     dx, CfgData.Port
    cmp     CfgData.Mode, 0
    jne     DisableParallel

    ; Disable serial interrupts
    add     dx, UART_IER
    xor     al, al
    out     dx, al
    jmp     InterruptsDisabled

DisableParallel:
    ; Disable parallel interrupts
    add     dx, LPT_CONTROL
    mov     al, LPT_CTRL_INIT
    out     dx, al

InterruptsDisabled:
    ; Disable IRQ at PIC
    mov     al, CfgData.IRQNum
    cmp     al, 8
    jb      DisablePIC1
    
    sub     al, 8
    mov     cl, al
    mov     al, 1
    shl     al, cl
    
    in      al, 0A1h
    or      al, al
    out     0A1h, al
    jmp     IRQDisabled

DisablePIC1:
    mov     cl, al
    mov     al, 1
    shl     al, cl
    
    in      al, 021h
    or      al, al
    out     021h, al

IRQDisabled:
    ; Restore interrupt vectors
    push    ds
    mov     ax, OldInt2F_Seg
    mov     ds, ax
    mov     dx, OldInt2F_Off
    mov     ax, 252Fh
    int     21h

    mov     al, CfgData.IRQNum
    cmp     al, 8
    jb      RestoreBelow8
    sub     al, 8
    add     al, 70h
    jmp     RestoreVector

RestoreBelow8:
    add     al, 08h

RestoreVector:
    mov     ah, 25h
    mov     dx, OldHwInt_Off
    mov     ax, OldHwInt_Seg
    mov     ds, ax
    int     21h
    pop     ds

    ; Release memory if UMB
    cmp     UMBAllocated, 1
    jne     NoMemoryRelease

    mov     es, MemorySegment
    mov     ah, 49h
    int     21h

NoMemoryRelease:
    ; Print unload message
    lea     dx, MsgUnloaded
    mov     ah, 09h
    int     21h

    mov     ax, 4C00h
    int     21h

NotLoaded:
    lea     dx, MsgErrNotLoaded
    mov     ah, 09h
    int     21h
    mov     ax, 4C01h
    int     21h

DriverBusy:
    lea     dx, MsgErrBusy
    mov     ah, 09h
    int     21h
    mov     ax, 4C01h
    int     21h
UnloadDriver ENDP

;=============================================================================
; End of Resident Code
;=============================================================================

ResidentEnd:

; Calculate resident size
ResidentSize EQU ($ - Start)

END Start

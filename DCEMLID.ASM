; DCEMLID.ASM - ODI MLID Driver for Serial/Parallel Direct Cable Connection
; Implements point-to-point IPX networking over null-modem or parallel cables
; Supports configurable MAC, baud rates, flow control, and automatic UMB loading
; 8086 CPU compatible (runs on IBM PC/XT and all compatibles)

; Build as .COM file with: TASM /m2 /jJUMPS DCEMLID.ASM && TLINK /t DCEMLID.OBJ
; Home of sourcecode: https://github.com/smarthomerocks/dcemlid
;
; For specification of LSL, MLID, ODI and IPX,see: dos_pstack.txt



.8086                           ; 8086 CPU instructions only

CODE    SEGMENT
        ASSUME CS:CODE, DS:CODE, ES:CODE, SS:CODE
        ORG 100h                ; .COM file starts at offset 100h

;=============================================================================
; Constants and Equates
;=============================================================================

; Driver Information
DRIVER_VERSION  EQU 0100h               ; Version 1.00
MTU_SIZE        EQU 576                 ; Maximum Transmission Unit (IPX max)

; Multiplex and LSL Constants
MULTIPLEX_ID    EQU 0C0A0h              ; Our multiplex ID
LSL_CHECK       EQU 0C000h              ; Check if LSL loaded
LSL_REGISTER    EQU 0C001h              ; Register MLID with LSL
LSL_GET_CONFIG  EQU 0C003h              ; Get configuration from LSL

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

; ECB (Event Control Block) Structure Offsets
ECB_Link        EQU 0                   ; DD - Link to next ECB
ECB_ESR         EQU 4                   ; DD - Event Service Routine
ECB_InUse       EQU 8                   ; DB - In use flag
ECB_CompCode    EQU 9                   ; DB - Completion code
ECB_Socket      EQU 10                  ; DW - Socket number
ECB_IPXWork     EQU 12                  ; 4 bytes - IPX workspace
ECB_DriverWork  EQU 16                  ; 12 bytes - Driver workspace
ECB_ImmAddr     EQU 28                  ; 6 bytes - Immediate address
ECB_FragCount   EQU 34                  ; DW - Fragment count
ECB_FragList    EQU 36                  ; Fragment descriptors start

; Fragment Descriptor Offsets (each 6 bytes)
FRAG_Addr       EQU 0                   ; DD - Fragment address
FRAG_Size       EQU 4                   ; DW - Fragment size

; ECB Completion Codes
ECB_SUCCESS     EQU 00h                 ; Successful
ECB_CANCELLED   EQU 0FCh                ; Cancelled by application
ECB_HARDWARE    EQU 0FDh                ; Hardware error
ECB_BAD_PKT     EQU 0FEh                ; Malformed packet
ECB_OVERFLOW    EQU 0FFh                ; Packet overflow

;=============================================================================
; Entry Point - Jump over data section
;=============================================================================

Start:
        jmp     Main            ; Skip over data section to main code

;=============================================================================
; Data Section (embedded in code segment for .COM file)
;=============================================================================

; Driver Information
DRIVER_NAME     DB 'DCEMLID',0

; Configuration Parameters
Config STRUC
    Mode        DB 0                    ; 0=Serial, 1=PAR4, 2=PAR8
    Port        DW 3F8h                 ; I/O port base address
    BaudRate    DD 115200               ; Baud rate (serial only)
    FlowCtrl    DB 0                    ; 0=RTS/CTS, 1=XON/XOFF
    MacAddr     DB 6 DUP(0)             ; MAC address
    IRQNum      DB 0                    ; IRQ number
    UARTType    DB 0                    ; 0=8250, 1=16550
    ParMode     DB 0                    ; For PAR8: 0=EPP, 1=PS/2
    BoardNum    DW 0                    ; Board number for multiple instances
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
RxHandlerOff    DW 0                    ; LSL receive handler offset
RxHandlerSeg    DW 0                    ; LSL receive handler segment

; LSL Entry Point (read from LSL Support Table)
LSLEntryOff     DW 0
LSLEntrySeg     DW 0
LSLVersion      DW 0                    ; LSL version (AL=major, AH=minor)
LSLBoardNum     DW 0                    ; Board number assigned by LSL

; Interrupt Vectors (Off then Seg for correct FAR JMP dword ptr)
OldInt2F_Off    DW 0
OldInt2F_Seg    DW 0
OldHwInt_Off    DW 0
OldHwInt_Seg    DW 0

; Transmit State
TxSeqNum        DB 0                    ; Current TX sequence number
TxPending       DB 0                    ; TX in progress flag
TxRetryCount    DB 0                    ; Current retry count
TxTimeout       DW 0                    ; Timeout counter

; ECB Queue Management
TxQueueHead     DD 0                    ; Head of transmit ECB queue
TxQueueTail     DD 0                    ; Tail of transmit ECB queue
TxQueueCount    DW 0                    ; Number of ECBs in queue
TxCurrentECB    DD 0                    ; Currently transmitting ECB
TxCurrentSeg    DW 0                    ; Segment of current ECB
TxCurrentOff    DW 0                    ; Offset of current ECB
TxFragIndex     DW 0                    ; Current fragment index
TxFragOffset    DW 0                    ; Offset within current fragment
TxTotalSent     DW 0                    ; Total bytes sent in current packet
TxPacketSize    DW 0                    ; Total packet size
TxCRC           DW 0FFFFh               ; Transmit CRC

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
FrameBuffer     DB MTU_SIZE+32 DUP(0)   ; Temp frame assembly buffer (extra for headers)
TxFrameBuffer   DB MTU_SIZE+32 DUP(0)   ; Transmit frame assembly buffer

; LSL Configuration Keywords
CfgKeyMode      DB 'MODE',0
CfgKeyPort      DB 'PORT',0
CfgKeyInt       DB 'INT',0
CfgKeyBaud      DB 'BAUD',0
CfgKeyFlow      DB 'FLOW',0
CfgKeyNode      DB 'NODE',0
CfgValSerial    DB 'SERIAL',0
CfgValPar4      DB 'PAR4',0
CfgValPar8      DB 'PAR8',0
CfgValRTS       DB 'RTS',0
CfgValXON       DB 'XON',0

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
MsgFlowRTS      DB ' Flow=RTS/CTS$'
MsgFlowXON      DB ' Flow=XON/XOFF$'
MsgUMB          DB ' [UMB]$'
MsgConv         DB ' [Conventional]$'
MsgCRLF         DB 13,10,'$'

MsgWarn8250     DB 'WARNING: 8250 UART at high baud - packet loss may occur.',13,10
                DB '         Consider using 38400 or lower.',13,10,'$'

MsgErrLSL       DB 'ERROR: LSL not loaded. Load LSL.COM first.',13,10,'$'
MsgErrConfig    DB 'ERROR: No configuration in NET.CFG. Add Link Driver DCEMLID section.',13,10,'$'
MsgErrLoaded    DB 'ERROR: Driver already loaded.',13,10,'$'
MsgErrIRQ       DB 'ERROR: IRQ conflict detected.',13,10,'$'
MsgErrPort      DB 'ERROR: Hardware not detected at specified port.',13,10,'$'
MsgErrDOS       DB 'ERROR: Requires DOS 5.0 or higher for UMB support.',13,10,'$'
MsgErrLSLVer    DB 'ERROR: Incompatible LSL version. Requires LSL 2.x, found: $'

MsgDbgPort      DB 'Detecting hardware at port $'
MsgDbgMode      DB ' mode=$'
MsgDbgSerial    DB 'SERIAL$'
MsgDbgPar4      DB 'PAR4$'
MsgDbgPar8      DB 'PAR8$'

MsgDbgLCR       DB 'LCR port=$'
MsgDbgWrite     DB ' write=$'
MsgDbgRead      DB ' read=$'
MsgDbgOK        DB ' OK',13,10,'$'
MsgDbgFAIL      DB ' FAIL',13,10,'$'
MsgDbgTest1     DB 'Test1:$'
MsgDbgTest2     DB 'Test2:$'
MsgDbgHwInit    DB 'Hardware init OK',13,10,'$'
MsgDbgHookIRQ   DB 'Hooking IRQ...$'
MsgDbgRegLSL    DB 'Registering with LSL...$'
MsgDbgLSLFail   DB ' failed (AX=$'
MsgDbgLSLRet    DB ')',13,10,'$'
MsgDbgLSLVer    DB 'LSL version=$'
MsgDbgCallLSL   DB 'Calling LSL RegisterMLID...',13,10,'$'

MsgErrLSLReg    DB 'ERROR: LSL registration failed.',13,10,'$'

MsgUnloaded     DB 'Driver unloaded successfully.',13,10,'$'
MsgErrNotLoaded DB 'ERROR: Driver not loaded.',13,10,'$'
MsgErrBusy      DB 'ERROR: Driver busy, cannot unload.',13,10,'$'
MsgErrNotLast   DB 'ERROR: Another TSR loaded after driver. Unload it first.',13,10,'$'

; Unload helper variables (static, not LOCAL for .COM compatibility)
Unload_TSRSeg       DW 0
Unload_TSRBoardNum  DW 0
Unload_TSRPort      DW 0
Unload_TSRMode      DB 0
Unload_TSRIRQNum    DB 0

; Help Screen
HelpText        DB 13,10,'DCEMLID v1.00 - ODI MLID Driver for Direct Cable Connection',13,10
                DB 13,10,'Usage: DCEMLID [/D] [/U] [/?]',13,10
                DB 13,10,'  /D  Enable debug output during hardware detection',13,10
                DB '  /U  Unload driver from memory',13,10
                DB '  /?  Show this help',13,10
                DB 13,10,'Configuration is read from NET.CFG. Example:',13,10
                DB 13,10,'  Link Driver DCEMLID',13,10
                DB '     PORT 3F8',13,10
                DB '     INT 4',13,10
                DB '     MODE SERIAL',13,10
                DB '     BAUD 115200',13,10
                DB '     FLOW RTS',13,10
                DB '     NODE ADDRESS 02608C123456',13,10
                DB 13,10,'For multiple instances, specify board number:',13,10
                DB '  Link Driver DCEMLID 0',13,10
                DB '  Link Driver DCEMLID 1',13,10
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

; Command Line Options (only help and unload)
OptUnload       DB '/U',0
OptHelp         DB '/?',0
OptDebug        DB '/D',0

; Baud Rate Table (divisor values for 115200 base)
BaudTableSize EQU 5                  ; Number of baud rate entries
BaudTable:
    DD 9600
    DW 12                    ; 115200 / 12 = 9600
    DD 19200
    DW 6                     ; 115200 / 6 = 19200
    DD 38400
    DW 3                     ; 115200 / 3 = 38400
    DD 57600
    DW 2                     ; 115200 / 2 = 57600
    DD 115200
    DW 1                     ; 115200 / 1 = 115200
    DD 0                     ; End marker

;=============================================================================
; Main Code Section
;=============================================================================

Main:
        ; All segment registers already point to CS in .COM file
        ; (DOS sets CS=DS=ES=SS before running .COM)

    ; Display banner
    lea     dx, MsgBanner
    mov     ah, 09h
    int     21h

    ; Parse command line options FIRST (before any LSL interaction)
    call    ParseCommandLine

    ; Check for help option - must work even if driver not loaded
    cmp     byte ptr [CmdHelp], 1
    je      ShowHelp

    ; Check for unload option - must work when driver IS loaded
    cmp     byte ptr [CmdUnload], 1
    je      UnloadDriver

    ; Get configuration from LSL/NET.CFG (only for installation)
    call    GetConfigFromLSL
    jc      ExitError

    ; Install driver
    call    InstallDriver
    jc      ExitError

    ; Stay resident
    mov     ax, 3100h           ; TSR with return code 0
    mov     dx, ResidentSize
    add     dx, 15
    mov     cl, 4
    shr     dx, cl              ; Convert bytes to paragraphs
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
; ParseCommandLine - Parse command line options only
; Sets CmdHelp, CmdUnload, CmdDebug flags
;=============================================================================

ParseCommandLine PROC
    push    ax
    push    cx
    push    si
    
    ; Get command line from PSP
    mov     si, 80h
    xor     cx, cx
    mov     cl, [si]
    inc     si
    jcxz    ParseCmdDone
    
ParseCmdLoop:
    ; Skip spaces
    call    SkipSpaces
    jcxz    ParseCmdDone
    
    ; Check for option marker
    cmp     byte ptr [si], '/'
    jne     ParseCmdNext
    
    ; Found option, check type
    inc     si
    dec     cx
    jcxz    ParseCmdDone
    
    mov     al, [si]
    ; Convert to uppercase
    cmp     al, 'a'
    jb      ParseCmdCheckOpt
    cmp     al, 'z'
    ja      ParseCmdCheckOpt
    sub     al, 20h
    
ParseCmdCheckOpt:
    cmp     al, '?'
    je      SetHelpFlag
    cmp     al, 'U'
    je      SetUnloadFlag
    cmp     al, 'D'
    je      SetDebugFlag
    jmp     ParseCmdNext
    
SetHelpFlag:
    mov     CmdHelp, 1
    jmp     ParseCmdNext
    
SetUnloadFlag:
    mov     CmdUnload, 1
    jmp     ParseCmdNext
    
SetDebugFlag:
    mov     CmdDebug, 1
    jmp     ParseCmdNext
    
ParseCmdNext:
    inc     si
    dec     cx
    jnz     ParseCmdLoop
    
ParseCmdDone:
    pop     si
    pop     cx
    pop     ax
    ret
ParseCommandLine ENDP

;=============================================================================
; GetConfigFromLSL - Retrieve configuration from LSL/NET.CFG
; Input: None
; Output: CF=1 on error, CF=0 on success
;=============================================================================

CmdHelp         DB 0
CmdUnload       DB 0
CmdDebug        DB 0

GetConfigFromLSL PROC
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di
    push    es

    ; Initialize defaults
    mov     CfgData.Mode, 0             ; Serial
    mov     CfgData.Port, 3F8h          ; COM1
    mov     word ptr CfgData.BaudRate, 0C200h     ; 115200 low word
    mov     word ptr CfgData.BaudRate+2, 1        ; 115200 high word
    mov     CfgData.FlowCtrl, 0         ; RTS/CTS
    mov     CfgData.BoardNum, 0         ; First instance
    
    ; Generate random MAC from BIOS timer (may be overridden)
    call    GenerateRandomMAC

    ; Get board number from command line if specified
    mov     si, 80h
    xor     cx, cx
    mov     cl, [si]
    inc     si
    jcxz    NoBoardNum
    
    ; Skip spaces and options to find board number
    call    SkipSpaces
    jcxz    NoBoardNum

ParseCmdArg:
    ; Skip options (starting with /)
    cmp     byte ptr [si], '/'
    jne     TryBoardNum
    ; Skip this option
    inc     si
    dec     cx
    jcxz    NoBoardNum
    inc     si
    dec     cx
    jcxz    NoBoardNum
    call    SkipSpaces
    jcxz    NoBoardNum
    jmp     ParseCmdArg

TryBoardNum:
    ; Parse board number (0-9)
    mov     al, [si]
    cmp     al, '0'
    jb      NoBoardNum
    cmp     al, '9'
    ja      NoBoardNum
    sub     al, '0'
    xor     ah, ah
    mov     CfgData.BoardNum, ax

NoBoardNum:
    ; Call LSL to get configuration
    ; AX = LSL_GET_CONFIG (0xC003)
    ; BX = Board number
    ; ES:SI = Driver name
    mov     ax, LSL_GET_CONFIG
    mov     bx, CfgData.BoardNum
    push    ds
    pop     es
    lea     si, DRIVER_NAME
    int     2Fh
    
    ; Check if configuration found
    cmp     al, 0
    jne     NoConfig
    
    ; ES:SI now points to configuration block from NET.CFG
    ; Parse configuration keywords
    call    ParseLSLConfig
    jc      ConfigError

ConfigDone:
    clc
    pop     es
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

NoConfig:
    ; No configuration in NET.CFG, use defaults
    clc
    jmp     ConfigDone

ConfigError:
    stc
    pop     es
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret
GetConfigFromLSL ENDP

;=============================================================================
; ParseLSLConfig - Parse LSL configuration block
; Input: ES:SI = configuration string from LSL
; Output: CF=1 on error, CF=0 on success
;=============================================================================

ParseLSLConfig PROC
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di

ParseLSLLoop:
    ; Check for end of configuration
    mov     al, es:[si]
    test    al, al
    jz      ParseLSLDone
    
    ; Skip spaces
SkipLSLSpaces:
    cmp     byte ptr es:[si], ' '
    jne     NotSpace
    inc     si
    jmp     SkipLSLSpaces
NotSpace:
    
    ; Check for PORT keyword
    lea     di, CfgKeyPort
    call    CompareLSLKeyword
    je      ParseLSLPort
    
    ; Check for INT keyword
    lea     di, CfgKeyInt
    call    CompareLSLKeyword
    je      ParseLSLInt
    
    ; Check for MODE keyword
    lea     di, CfgKeyMode
    call    CompareLSLKeyword
    je      ParseLSLMode
    
    ; Check for BAUD keyword
    lea     di, CfgKeyBaud
    call    CompareLSLKeyword
    je      ParseLSLBaud
    
    ; Check for FLOW keyword
    lea     di, CfgKeyFlow
    call    CompareLSLKeyword
    je      ParseLSLFlow
    
    ; Check for NODE keyword
    lea     di, CfgKeyNode
    call    CompareLSLKeyword
    je      ParseLSLNode
    
    ; Unknown keyword, skip to end of line
    call    SkipToLSLEOL
    jmp     ParseLSLLoop

ParseLSLPort:
    call    ParseLSLHexValue
    jc      ParseLSLLoop
    mov     CfgData.Port, ax
    jmp     ParseLSLLoop

ParseLSLInt:
    call    ParseLSLDecValue
    jc      ParseLSLLoop
    mov     CfgData.IRQNum, al
    jmp     ParseLSLLoop

ParseLSLMode:
    lea     di, CfgValSerial
    call    CompareLSLValue
    je      SetModeSerial
    lea     di, CfgValPar4
    call    CompareLSLValue
    je      SetModePar4
    lea     di, CfgValPar8
    call    CompareLSLValue
    je      SetModePar8
    jmp     ParseLSLLoop

SetModeSerial:
    mov     CfgData.Mode, 0
    jmp     ParseLSLLoop

SetModePar4:
    mov     CfgData.Mode, 1
    mov     CfgData.Port, 378h          ; Default LPT1
    jmp     ParseLSLLoop

SetModePar8:
    mov     CfgData.Mode, 2
    mov     CfgData.Port, 378h          ; Default LPT1
    jmp     ParseLSLLoop

ParseLSLBaud:
    call    ParseLSLDecValue
    jc      ParseLSLLoop
    mov     word ptr CfgData.BaudRate, ax
    mov     word ptr CfgData.BaudRate+2, 0
    jmp     ParseLSLLoop

ParseLSLFlow:
    lea     di, CfgValRTS
    call    CompareLSLValue
    je      SetFlowRTS
    lea     di, CfgValXON
    call    CompareLSLValue
    je      SetFlowXON
    jmp     ParseLSLLoop

SetFlowRTS:
    mov     CfgData.FlowCtrl, 0
    jmp     ParseLSLLoop

SetFlowXON:
    mov     CfgData.FlowCtrl, 1
    jmp     ParseLSLLoop

ParseLSLNode:
    ; Parse NODE ADDRESS xxxxxxxxxxxx (12 hex digits)
    ; Skip "ADDRESS" keyword
    call    SkipLSLWord
    ; Parse 12 hex digits for MAC address
    lea     di, CfgData.MacAddr
    mov     cx, 6
ParseLSLNodeLoop:
    call    ParseLSLHexByte
    jc      ParseLSLLoop
    stosb
    loop    ParseLSLNodeLoop
    jmp     ParseLSLLoop

ParseLSLDone:
    clc
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret
ParseLSLConfig ENDP

; Helper: Compare keyword at ES:SI with DS:DI
CompareLSLKeyword PROC
    push    si
    push    di
CompLSLKeyLoop:
    mov     al, [di]
    test    al, al
    jz      CompLSLKeyMatch
    mov     ah, es:[si]
    call    ToUpper
    xchg    al, ah
    call    ToUpper
    cmp     al, ah
    jne     CompLSLKeyNoMatch
    inc     si
    inc     di
    jmp     CompLSLKeyLoop
CompLSLKeyMatch:
    pop     di
    pop     si
    ; Advance SI past keyword
    add     si, di
    sub     si, offset CfgKeyMode
    xor     al, al              ; Set ZF
    ret
CompLSLKeyNoMatch:
    pop     di
    pop     si
    or      al, 1               ; Clear ZF
    ret
CompareLSLKeyword ENDP

; Helper: Compare value at ES:SI with DS:DI
CompareLSLValue PROC
    push    si
    push    di
CompLSLValLoop:
    mov     al, [di]
    test    al, al
    jz      CompLSLValMatch
    mov     ah, es:[si]
    cmp     ah, ' '
    je      CompLSLValMatch
    cmp     ah, 13
    je      CompLSLValMatch
    cmp     ah, 10
    je      CompLSLValMatch
    call    ToUpper
    xchg    al, ah
    call    ToUpper
    cmp     al, ah
    jne     CompLSLValNoMatch
    inc     si
    inc     di
    jmp     CompLSLValLoop
CompLSLValMatch:
    pop     di
    pop     si
    add     si, di
    sub     si, offset CfgValSerial
    xor     al, al
    ret
CompLSLValNoMatch:
    pop     di
    pop     si
    or      al, 1
    ret
CompareLSLValue ENDP

; Helper: Parse hex value from ES:SI
ParseLSLHexValue PROC
    push    bx
    xor     bx, bx
    mov     cx, 4
ParseLSLHexLoop:
    mov     al, es:[si]
    call    HexDigitToValue
    jc      ParseLSLHexDone
    mov     cl, 4
    shl     bx, cl
    or      bl, al
    inc     si
    loop    ParseLSLHexLoop
ParseLSLHexDone:
    mov     ax, bx
    clc
    pop     bx
    ret
ParseLSLHexValue ENDP

; Helper: Parse decimal value from ES:SI
ParseLSLDecValue PROC
    push    bx
    push    dx
    xor     bx, bx
ParseLSLDecLoop:
    mov     al, es:[si]
    cmp     al, '0'
    jb      ParseLSLDecDone
    cmp     al, '9'
    ja      ParseLSLDecDone
    sub     al, '0'
    mov     dx, 10
    push    ax
    mov     ax, bx
    mul     dx
    mov     bx, ax
    pop     ax
    add     bx, ax
    inc     si
    jmp     ParseLSLDecLoop
ParseLSLDecDone:
    mov     ax, bx
    clc
    pop     dx
    pop     bx
    ret
ParseLSLDecValue ENDP

; Helper: Parse hex byte from ES:SI
ParseLSLHexByte PROC
    push    bx
    xor     ah, ah
    mov     bl, 2
ParseLSLHexByteLoop:
    mov     al, es:[si]
    call    HexDigitToValue
    jc      ParseLSLHexByteDone
    mov     cl, 4
    shl     ah, cl
    or      ah, al
    inc     si
    dec     bl
    jnz     ParseLSLHexByteLoop
ParseLSLHexByteDone:
    mov     al, ah
    clc
    pop     bx
    ret
ParseLSLHexByte ENDP

; Helper: Skip to end of line
SkipToLSLEOL PROC
SkipLSLEOLLoop:
    mov     al, es:[si]
    cmp     al, 13
    je      SkipLSLEOLDone
    cmp     al, 10
    je      SkipLSLEOLDone
    test    al, al
    jz      SkipLSLEOLDone
    inc     si
    jmp     SkipLSLEOLLoop
SkipLSLEOLDone:
    ret
SkipToLSLEOL ENDP

; Helper: Skip one word
SkipLSLWord PROC
SkipLSLWordLoop:
    mov     al, es:[si]
    cmp     al, ' '
    je      SkipLSLWordDone
    cmp     al, 13
    je      SkipLSLWordDone
    cmp     al, 10
    je      SkipLSLWordDone
    test    al, al
    jz      SkipLSLWordDone
    inc     si
    jmp     SkipLSLWordLoop
SkipLSLWordDone:
    ret
SkipLSLWord ENDP

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
    
    push    cx              ; Save outer loop counter
    mov     cl, 4
    shl     bx, cl
    pop     cx              ; Restore outer loop counter
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
    
    push    cx              ; Save outer loop counter
    mov     cl, 4
    shl     ah, cl
    pop     cx              ; Restore outer loop counter
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
    
    ; LSL 2.x returns:
    ; AL = FFh (loaded)
    ; ES:BX = pointer to LSL function table (entry point)
    ; CX or DX may contain version in some implementations
    ;
    ; Save registers immediately before any debug code modifies them
    mov     LSLEntrySeg, es
    mov     LSLEntryOff, bx
    
    ; For LSL 2.x, ES:BX IS the entry point (function table)
    ; Skip version detection - we verified LSL is loaded (AL=FFh)
    ; Version checking removed since we can't reliably detect it
    mov     LSLVersion, 0200h   ; Assume 2.0 compatible

    ; Skip version check - we assume LSL is compatible if it responds to C000h
    ; Version detection is unreliable across different LSL implementations

    ; Check DOS version for UMB support
    mov     ah, 30h
    int     21h
    cmp     al, 5
    jb      DOSTooOld

    ; Print settings before hardware detection
    call    PrintDetectionInfo

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
    
    ; Debug: hardware init completed
    cmp     CmdDebug, 1
    jne     SkipDbgHwInit
    push    dx
    lea     dx, MsgDbgHwInit
    mov     ah, 09h
    int     21h
    pop     dx
SkipDbgHwInit:
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

    ; Debug: hooking IRQ
    cmp     CmdDebug, 1
    jne     SkipDbgHookIRQ
    push    dx
    lea     dx, MsgDbgHookIRQ
    mov     ah, 09h
    int     21h
    pop     dx
SkipDbgHookIRQ:

    ; Hook hardware interrupt
    call    HookHardwareInterrupt
    jc      InitError
    
    ; Debug: print OK
    cmp     CmdDebug, 1
    jne     SkipDbgIRQOK
    push    dx
    lea     dx, MsgDbgOK
    mov     ah, 09h
    int     21h
    pop     dx
SkipDbgIRQOK:

    ; Debug: registering with LSL
    cmp     CmdDebug, 1
    jne     SkipDbgRegLSL
    push    dx
    lea     dx, MsgDbgRegLSL
    mov     ah, 09h
    int     21h
    pop     dx
SkipDbgRegLSL:

    ; Register with LSL
    call    RegisterWithLSL
    jc      InitErrorLSL

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

LSLIncompat:
    lea     dx, MsgErrLSLVer
    mov     ah, 09h
    int     21h
    ; Print detected version (LSLVersion: AL=major, AH=minor)
    mov     ax, LSLVersion
    push    ax
    xor     ah, ah              ; AL = Major
    call    PrintDecimal
    mov     dl, '.'
    mov     ah, 02h
    int     21h
    pop     ax
    mov     al, ah              ; Minor is in AH
    xor     ah, ah
    call    PrintDecimal
    lea     dx, MsgCRLF
    mov     ah, 09h
    int     21h
    stc
    ret

InitError:
    ; Restore interrupt vectors if needed
    cmp     OldInt2F_Off, 0
    je      InitErrorNoRestore
    
    ; Load both offset and segment BEFORE changing DS
    mov     dx, OldInt2F_Off
    push    ds
    mov     ax, OldInt2F_Seg
    mov     ds, ax
    mov     ax, 252Fh
    int     21h
    pop     ds

InitErrorNoRestore:
    lea     dx, MsgErrPort
    mov     ah, 09h
    int     21h
    stc
    ret

InitErrorLSL:
    ; Restore interrupt vectors if needed
    cmp     OldInt2F_Off, 0
    je      InitErrorLSLNoRestore
    
    mov     dx, OldInt2F_Off
    push    ds
    mov     ax, OldInt2F_Seg
    mov     ds, ax
    mov     ax, 252Fh
    int     21h
    pop     ds

InitErrorLSLNoRestore:
    lea     dx, MsgErrLSLReg
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
    push    cx

    mov     dx, CfgData.Port

    ; === UART Detection using Line Control Register (LCR) test ===
    ; The original 8250 does NOT have a scratch register (offset 7),
    ; so we use the LCR (offset 3) instead for reliable detection.
    ; Save current LCR, write test patterns, then restore.
    add     dx, UART_LCR        ; Line Control Register
    
    ; Debug: print LCR port address
    cmp     CmdDebug, 1
    jne     SkipDbgLCR
    push    ax
    push    dx                  ; Save LCR port address
    push    dx                  ; Will use this for PrintHexWord
    lea     dx, MsgDbgLCR
    mov     ah, 09h
    int     21h
    pop     ax                  ; Get port address into AX for printing
    call    PrintHexWord
    lea     dx, MsgCRLF
    mov     ah, 09h
    int     21h
    pop     dx                  ; Restore LCR port address
    pop     ax
SkipDbgLCR:
    
    in      al, dx              ; Save current LCR value
    mov     ah, al              ; Store in AH
    
    ; Test pattern 1
    mov     al, 01Bh            ; Test pattern 1 (8N1 + break)
    out     dx, al
    push    ax                  ; Save written value
    in      al, dx
    mov     bl, al              ; Save raw read
    and     al, 07Fh            ; Mask off DLAB bit
    mov     bh, al              ; Save masked read
    pop     ax                  ; Restore written value
    
    ; Debug: print test 1 results
    cmp     CmdDebug, 1
    jne     SkipDbgTest1
    push    ax
    push    bx
    push    dx
    lea     dx, MsgDbgTest1
    mov     ah, 09h
    int     21h
    lea     dx, MsgDbgWrite
    mov     ah, 09h
    int     21h
    mov     al, 01Bh
    call    PrintHexByte
    lea     dx, MsgDbgRead
    mov     ah, 09h
    int     21h
    pop     dx
    pop     bx
    push    bx
    push    dx
    mov     al, bl              ; Raw read value
    call    PrintHexByte
    mov     dl, '('
    mov     ah, 02h
    int     21h
    mov     al, bh              ; Masked value
    call    PrintHexByte
    mov     dl, ')'
    mov     ah, 02h
    int     21h
    pop     dx
    pop     bx
    pop     ax
SkipDbgTest1:
    
    cmp     bh, 01Bh
    jne     NoUARTFoundDbg
    
    ; Debug: print OK
    cmp     CmdDebug, 1
    jne     SkipDbgOK1
    push    dx
    lea     dx, MsgDbgOK
    mov     ah, 09h
    int     21h
    pop     dx
SkipDbgOK1:
    
    ; Test pattern 2
    mov     al, 003h            ; Test pattern 2 (8N1)
    out     dx, al
    push    ax
    in      al, dx
    mov     bl, al              ; Save raw read
    and     al, 07Fh
    mov     bh, al              ; Save masked read
    pop     ax
    
    ; Debug: print test 2 results
    cmp     CmdDebug, 1
    jne     SkipDbgTest2
    push    ax
    push    bx
    push    dx
    lea     dx, MsgDbgTest2
    mov     ah, 09h
    int     21h
    lea     dx, MsgDbgWrite
    mov     ah, 09h
    int     21h
    mov     al, 003h
    call    PrintHexByte
    lea     dx, MsgDbgRead
    mov     ah, 09h
    int     21h
    pop     dx
    pop     bx
    push    bx
    push    dx
    mov     al, bl
    call    PrintHexByte
    mov     dl, '('
    mov     ah, 02h
    int     21h
    mov     al, bh
    call    PrintHexByte
    mov     dl, ')'
    mov     ah, 02h
    int     21h
    pop     dx
    pop     bx
    pop     ax
SkipDbgTest2:
    
    cmp     bh, 003h
    jne     NoUARTFoundDbg
    
    ; Debug: print OK
    cmp     CmdDebug, 1
    jne     SkipDbgOK2
    push    dx
    lea     dx, MsgDbgOK
    mov     ah, 09h
    int     21h
    pop     dx
SkipDbgOK2:
    
    ; Restore original LCR
    mov     al, ah
    out     dx, al
    jmp     UARTDetected

NoUARTFoundDbg:
    ; Debug: print FAIL
    cmp     CmdDebug, 1
    jne     NoUARTFound
    push    dx
    lea     dx, MsgDbgFAIL
    mov     ah, 09h
    int     21h
    pop     dx
    jmp     NoUARTFound

UARTDetected:
    
    ; UART detected, continue with initialization
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
    
    ; Check if baud rate is too high for 8250 (compare low word, high word is 0 for valid rates)
    cmp     word ptr CfgData.BaudRate, 57600
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
    ; Look up divisor for baud rate from table (8086-compatible)
    call    LookupBaudDivisor
    ; BX now contains the divisor

    ; Set DLAB to access divisor latch
    mov     dx, CfgData.Port
    add     dx, UART_LCR
    mov     al, LCR_DLAB OR LCR_8N1
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
    mov     al, FCR_ENABLE OR FCR_CLEAR OR FCR_TRIGGER_8
    out     dx, al

NoFIFO:
    ; Configure modem control
    mov     dx, CfgData.Port
    add     dx, UART_MCR
    mov     al, MCR_DTR OR MCR_RTS OR MCR_OUT2
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
    pop     cx
    pop     ax
    pop     dx
    ret

NoUARTFound:
    ; No UART detected at this port
    stc
    pop     cx
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
    push    cx

    mov     dx, CfgData.Port

    ; === Parallel port detection using data register test ===
    ; Write test patterns and read back
    mov     al, 055h
    out     dx, al
    in      al, dx
    cmp     al, 055h
    jne     NoLPTFound
    
    mov     al, 0AAh
    out     dx, al
    in      al, dx
    cmp     al, 0AAh
    jne     NoLPTFound
    
    ; Parallel port detected, continue with initialization

    ; Check for PAR8 mode
    cmp     CfgData.Mode, 2
    jne     Par4Mode

    ; Try to detect EPP capability
    add     dx, LPT_ECR
    in      al, dx
    mov     cl, 5
    shr     al, cl              ; Get bits 5-7
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
    mov     al, LPT_CTRL_INIT OR LPT_CTRL_SELECT
    
    ; Add bidirectional bit for PS/2 mode
    cmp     CfgData.ParMode, 1
    jne     NoBidir
    or      al, LPT_CTRL_BIDIR

NoBidir:
    out     dx, al

    ; Parallel typically uses IRQ7
    mov     CfgData.IRQNum, 7

    clc
    pop     cx
    pop     ax
    pop     dx
    ret

NoLPTFound:
    ; No parallel port detected at this port
    stc
    pop     cx
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
    mov     cl, 4
    shr     bx, cl

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

; MLID Configuration Table for ODI 2.x (LSL 2.14) - Table 03002
; This is the full official ODI MLID configuration table format
MLIDConfig STRUC
    Signature               DB 26 DUP(?)    ; 00h: 'HardwareDriverMLID        ' (8 spaces)
    MConfigTableMajorVer    DB ?            ; 1Ah: Configuration table major version
    MConfigTableMinorVer    DB ?            ; 1Bh: Configuration table minor version
    NodeAddress             DB 6 DUP(?)     ; 1Ch: 6-byte node address
    ModeFlags               DW ?            ; 22h: Mode flags
    BoardNumber             DW ?            ; 24h: Board number (filled by LSL)
    BoardInstance           DW ?            ; 26h: Board instance
    MaxDataSize             DW ?            ; 28h: Maximum packet size
    BestDataSize            DW ?            ; 2Ah: Best data size
    WorstDataSize           DW ?            ; 2Ch: Worst data size
    NICLongName             DD ?            ; 2Eh: Far ptr to counted NUL-term long name
    NICShortName            DD ?            ; 32h: Far ptr to counted NUL-term short name
    FrameMediaType          DD ?            ; 36h: Far ptr to frame/media type string
    Reserved1               DW ?            ; 3Ah: Reserved (0000h)
    FrameTypeID             DW ?            ; 3Ch: Frame type ID
    TransportTime           DW ?            ; 3Eh: Transport time (ms)
    SourceRouteHandler      DD ?            ; 40h: Far ptr to source route handler
    LookAheadSize           DW ?            ; 44h: Lookahead size
    LineSpeed               DW ?            ; 46h: Line speed (Mbps or Kbps+8000h)
    QueueDepth              DW ?            ; 48h: Queue depth
    Reserved2               DB 6 DUP(?)     ; 4Ah: Reserved
    DriverMajorVer          DB ?            ; 50h: Driver major version
    DriverMinorVer          DB ?            ; 51h: Driver minor version
    BusMulticastFlags       DW ?            ; 52h: Bus/multicast flags
    SendRetries             DW ?            ; 54h: Send retries
    ConfigTableLink         DD ?            ; 56h: Config table link
    SharingFlags            DW ?            ; 5Ah: MLID sharing flags
    SlotNumber              DW ?            ; 5Ch: Slot number
    IOAddr1                 DW ?            ; 5Eh: I/O address 1
    IORange1                DW ?            ; 60h: I/O range 1
    IOAddr2                 DW ?            ; 62h: I/O address 2
    IORange2                DW ?            ; 64h: I/O range 2
    MemAddr1                DD ?            ; 66h: Memory address 1
    MemSize1                DW ?            ; 6Ah: Memory size 1
    MemAddr2                DD ?            ; 6Ch: Memory address 2
    MemSize2                DW ?            ; 70h: Memory size 2
    IRQLine1                DB ?            ; 72h: Interrupt line 1
    IRQLine2                DB ?            ; 73h: Interrupt line 2
    DMALine1                DB ?            ; 74h: DMA line 1
    DMALine2                DB ?            ; 75h: DMA line 2
MLIDConfig ENDS

; LSL Information Block returned from registration - Table 02991
LSLInfoBlock STRUC
    LSLEntryPoint           DD ?            ; 00h: Entry point (BX=function 00h-11h)
    LSLInfo1                DW ?            ; 04h: ???
    LSLInfo2                DW ?            ; 06h: ???
LSLInfoBlock ENDS

; MLID Info Block for LSL registration - Table 02990
; This wrapper is required for RegisterMLID (BX=0001h)
; ES:SI must point to this structure, NOT directly to the config table
MLIDInfoBlock STRUC
    MIBS_SendEntry          DD ?            ; 00h: Far ptr to MLID Send handler
    MIBS_ControlEntry       DD ?            ; 04h: Far ptr to MLID Control handler
    MIBS_ConfigTable        DD ?            ; 08h: Far ptr to MLID Config table
MLIDInfoBlock ENDS

; Interface Function Table for ODI 2.x
MLIDFuncTable STRUC
    GetStatistics           DD ?
    AddMulticast            DD ?
    DeleteMulticast         DD ?
    MLIDShutdown            DD ?
    MLIDReset               DD ?
    Reserved                DD 3 DUP(?)
MLIDFuncTable ENDS

; String data for registration - ODI uses "counted NUL-terminated" strings
; Format: length byte, string chars, NUL terminator
NICLongNameStr      DB 23,'DCEMLID Serial/Parallel',0
NICShortNameStr     DB 7,'DCEMLID',0
FrameMediaTypeStr   DB 14,'Ethernet_802.2',0  ; Frame/media type (14 chars)

; LSL info block buffer (filled by LSL on registration)
LSLInfoBuf          LSLInfoBlock <>

; MLID Info Block for registration (ES:SI points here)
MLIDInfoBlockData   MLIDInfoBlock <>

; Function table
MLIDFunctions   MLIDFuncTable <>

; MLID Configuration Table - initialized with signature
; Note: '>' is default initializer followed by per-field values
MLIDConfigTable MLIDConfig <'HardwareDriverMLID        ', 01h, 00h>

RegisterWithLSL PROC
    push    ax
    push    bx
    push    cx
    push    dx
    push    es
    push    di
    push    si

    ; Fill in configuration table
    ; Copy node address
    push    ds
    pop     es
    lea     si, CfgData.MacAddr
    lea     di, MLIDConfigTable.NodeAddress
    mov     cx, 6
    rep     movsb

    ; Set mode flags (bit 0 = RealDriverBit, always 1)
    mov     MLIDConfigTable.ModeFlags, 0001h
    
    ; Board instance from config
    mov     ax, CfgData.BoardNum
    mov     MLIDConfigTable.BoardInstance, ax
    
    ; Packet sizes
    mov     MLIDConfigTable.MaxDataSize, MTU_SIZE
    mov     MLIDConfigTable.BestDataSize, MTU_SIZE
    mov     MLIDConfigTable.WorstDataSize, 64
    
    ; Set string pointers (counted NUL-terminated strings)
    mov     ax, cs
    mov     word ptr MLIDConfigTable.NICLongName[2], ax
    lea     bx, NICLongNameStr
    mov     word ptr MLIDConfigTable.NICLongName[0], bx

    mov     word ptr MLIDConfigTable.NICShortName[2], ax
    lea     bx, NICShortNameStr
    mov     word ptr MLIDConfigTable.NICShortName[0], bx

    mov     word ptr MLIDConfigTable.FrameMediaType[2], ax
    lea     bx, FrameMediaTypeStr
    mov     word ptr MLIDConfigTable.FrameMediaType[0], bx
    
    ; Reserved field
    mov     MLIDConfigTable.Reserved1, 0
    
    ; Frame type (Ethernet_802.2 = 0005h, default for NetWare 3.12+)
    mov     MLIDConfigTable.FrameTypeID, 0005h
    
    ; Transport time, line speed
    mov     MLIDConfigTable.TransportTime, 1
    mov     MLIDConfigTable.LineSpeed, 8000h + 115  ; 115 Kbps (high bit = Kbps)
    mov     MLIDConfigTable.LookAheadSize, 64
    mov     MLIDConfigTable.QueueDepth, 4
    
    ; No source route handler
    mov     word ptr MLIDConfigTable.SourceRouteHandler[0], 0
    mov     word ptr MLIDConfigTable.SourceRouteHandler[2], 0
    
    ; Clear reserved bytes
    lea     di, MLIDConfigTable.Reserved2
    mov     cx, 6
    xor     al, al
    rep     stosb
    
    ; Driver version
    mov     MLIDConfigTable.DriverMajorVer, 1
    mov     MLIDConfigTable.DriverMinorVer, 0
    
    ; Bus/multicast flags (ISA card)
    mov     MLIDConfigTable.BusMulticastFlags, 0002h
    
    ; Send retries
    mov     MLIDConfigTable.SendRetries, 3
    
    ; Config table link (none)
    mov     word ptr MLIDConfigTable.ConfigTableLink[0], 0
    mov     word ptr MLIDConfigTable.ConfigTableLink[2], 0
    
    ; Sharing flags
    mov     MLIDConfigTable.SharingFlags, 0
    
    ; Slot number (N/A)
    mov     MLIDConfigTable.SlotNumber, 0
    
    ; I/O addresses from config
    mov     ax, CfgData.Port
    mov     MLIDConfigTable.IOAddr1, ax
    mov     MLIDConfigTable.IORange1, 8         ; 8 bytes for COM port
    mov     MLIDConfigTable.IOAddr2, 0
    mov     MLIDConfigTable.IORange2, 0
    
    ; Memory addresses (none)
    mov     word ptr MLIDConfigTable.MemAddr1[0], 0
    mov     word ptr MLIDConfigTable.MemAddr1[2], 0
    mov     MLIDConfigTable.MemSize1, 0
    mov     word ptr MLIDConfigTable.MemAddr2[0], 0
    mov     word ptr MLIDConfigTable.MemAddr2[2], 0
    mov     MLIDConfigTable.MemSize2, 0
    
    ; IRQ and DMA
    mov     al, CfgData.IRQNum
    mov     MLIDConfigTable.IRQLine1, al
    mov     MLIDConfigTable.IRQLine2, 0
    mov     MLIDConfigTable.DMALine1, 0
    mov     MLIDConfigTable.DMALine2, 0
    
    ; Set up interface function table
    mov     word ptr MLIDFunctions.GetStatistics[2], cs
    lea     bx, MLIDGetStats
    mov     word ptr MLIDFunctions.GetStatistics[0], bx
    
    mov     word ptr MLIDFunctions.AddMulticast[2], cs
    lea     bx, MLIDAddMulticast
    mov     word ptr MLIDFunctions.AddMulticast[0], bx
    
    mov     word ptr MLIDFunctions.DeleteMulticast[2], cs
    lea     bx, MLIDDelMulticast
    mov     word ptr MLIDFunctions.DeleteMulticast[0], bx
    
    mov     word ptr MLIDFunctions.MLIDShutdown[2], cs
    lea     bx, MLIDShutdownProc
    mov     word ptr MLIDFunctions.MLIDShutdown[0], bx
    
    mov     word ptr MLIDFunctions.MLIDReset[2], cs
    lea     bx, MLIDResetProc
    mov     word ptr MLIDFunctions.MLIDReset[0], bx

    ; Initialize MLIDInfoBlockData with function pointers and config table
    mov     word ptr MLIDInfoBlockData.MIBS_SendEntry[0], OFFSET MLIDTransmitHandler
    mov     word ptr MLIDInfoBlockData.MIBS_SendEntry[2], cs
    mov     word ptr MLIDInfoBlockData.MIBS_ControlEntry[0], OFFSET MLIDControlHandler
    mov     word ptr MLIDInfoBlockData.MIBS_ControlEntry[2], cs
    mov     word ptr MLIDInfoBlockData.MIBS_ConfigTable[0], OFFSET MLIDConfigTable
    mov     word ptr MLIDInfoBlockData.MIBS_ConfigTable[2], cs

    ; Register with LSL via entry point
    ; LSL 2.x calling convention (Table 02986, 02990):
    ; BX = function code (0001h = RegisterMLID)
    ; ES:SI = pointer to MLIDInfoBlock (SendEntry, ControlEntry, ConfigTable)
    ; DS:DI = buffer for LSL information block (Table 02991)
    ; Returns: AX = 0 on success, non-zero on error
    ;          BX = board number assigned by LSL
    push    ds
    pop     es
    lea     si, MLIDInfoBlockData   ; Point to info block wrapper, NOT config table
    lea     di, LSLInfoBuf
    
    ; Debug: print function call info
    cmp     CmdDebug, 1
    jne     SkipDbgCallLSL
    push    dx
    lea     dx, MsgDbgCallLSL
    mov     ah, 09h
    int     21h
    pop     dx
SkipDbgCallLSL:
    
    mov     bx, 0001h               ; Function 0001h: RegisterMLID (LSL 2.x)
    call    dword ptr [LSLEntryOff]

    ; Check return status
    cmp     ax, 0
    je      RegisterOK
    
    ; Debug: print failure and return code
    push    ax                  ; Save return code
    cmp     CmdDebug, 1
    jne     SkipDbgLSLFail
    push    dx
    lea     dx, MsgDbgLSLFail
    mov     ah, 09h
    int     21h
    pop     dx
    pop     ax
    push    ax
    call    PrintHexWord
    push    dx
    lea     dx, MsgDbgLSLRet
    mov     ah, 09h
    int     21h
    pop     dx
SkipDbgLSLFail:
    pop     ax
    stc
    jmp     RegisterDone

RegisterOK:
    ; Save board number assigned by LSL (returned in BX)
    mov     MLIDConfigTable.BoardNumber, bx
    mov     LSLBoardNum, bx
    
    ; Debug: print OK
    cmp     CmdDebug, 1
    jne     SkipDbgLSLOK
    push    dx
    lea     dx, MsgDbgOK
    mov     ah, 09h
    int     21h
    pop     dx
SkipDbgLSLOK:
    clc

RegisterDone:
    pop     si
    pop     di
    pop     es
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret
RegisterWithLSL ENDP

;=============================================================================
; MLID Interface Functions (called by LSL)
;=============================================================================

MLIDGetStats PROC FAR
    ; Return pointer to statistics in ES:SI
    push    ds
    pop     es
    lea     si, Statistics
    xor     ax, ax              ; Success
    retf
MLIDGetStats ENDP

MLIDAddMulticast PROC FAR
    ; Add multicast address - not supported for serial
    mov     ax, 1               ; Not supported
    retf
MLIDAddMulticast ENDP

MLIDDelMulticast PROC FAR
    ; Delete multicast address - not supported for serial
    mov     ax, 1               ; Not supported
    retf
MLIDDelMulticast ENDP

MLIDShutdownProc PROC FAR
    ; Shutdown the MLID
    ; Entry: AX = board number, CX = 0000h for permanent, non-zero for temporary
    ; NOTE: This just clears internal state. Deregistration from LSL must be
    ; done by UnloadDriver with proper segment setup.
    mov     DriverActive, 0
    xor     ax, ax
    retf
MLIDShutdownProc ENDP

MLIDResetProc PROC FAR
    ; Reset the MLID - reinitialize buffers
    mov     RxState, 0          ; STATE_IDLE = 0
    mov     RxBufIdx, 0
    mov     TxPending, 0
    xor     ax, ax
    retf
MLIDResetProc ENDP

;=============================================================================
; INT 2Fh Multiplex Handler
;=============================================================================

MULTIPLEX_AH    EQU (MULTIPLEX_ID SHR 8)    ; High byte of multiplex ID (C0h)

Int2FHandler PROC FAR
    cmp     ah, MULTIPLEX_AH        ; Check AH only, not full AX
    je      OurMultiplex
    
    ; Chain to previous handler
    jmp     dword ptr cs:[OldInt2F_Off]

OurMultiplex:
    ; AH = MULTIPLEX_ID high byte, AL = subfunction
    ; AL=00h/A0h: Installation check - return AL=FFh, ES=TSR segment
    ; AL=01h/A1h: Get info - return ES=TSR segment, BX=board number
    ; Check for subfunction 01h (either A1h or 01h depending on caller)
    test    al, 01h
    jnz     Mux_GetInfo
    
    ; Default: Installation check (AL=00h or A0h)
    mov     al, 0FFh
    push    cs
    pop     es                      ; Return TSR segment in ES
    iret

Mux_GetInfo:
    push    cs
    pop     es                      ; TSR segment in ES
    mov     bx, cs:[LSLBoardNum]    ; Board number in BX
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

    mov     ax, cs              ; .COM file: CS=DS=ES
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
    
    ; Check for ACK
    cmp     al, ACK_CHAR
    je      HandleACK
    
    ; Process received byte through frame parser
    call    ProcessReceivedByte
    jmp     SerialNotReceive

HandleACK:
    ; Read sequence number
    mov     dx, CfgData.Port
    push    ax
WaitForSeq:
    add     dx, UART_LSR
    in      al, dx
    test    al, 01h                 ; Data ready?
    jz      WaitForSeq
    sub     dx, UART_LSR
    in      al, dx                  ; Read sequence number
    
    ; Check if it matches current transmission
    cmp     al, byte ptr TxFrameBuffer
    jne     ACKMismatch
    
    ; ACK received, complete current ECB successfully
    push    es
    push    si
    les     si, dword ptr TxCurrentECB
    cmp     si, 0
    je      NoCurrentECB
    cmp     word ptr TxCurrentECB+2, 0
    je      NoCurrentECB
    mov     byte ptr es:[si+ECB_CompCode], ECB_SUCCESS
    call    CompleteTransmitECB
NoCurrentECB:
    pop     si
    pop     es
    
    ; Start next transmission
    call    StartNextTransmit
    
ACKMismatch:
    pop     ax

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

    mov     ax, cs              ; .COM file: CS=DS=ES
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
    lea     bx, Statistics.CRCErrors
    call    Inc32

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
    mov     cx, word ptr cs:CRCTable[bx]
    mov     ax, RxCRC
    mov     al, ah              ; shr ax, 8 - move high byte to low
    xor     ah, ah
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
    lea     bx, Statistics.RxPackets
    call    Inc32
    mov     ax, RxFrameLen
    call    ZeroExtend16to32    ; DX:AX = 32-bit length
    lea     bx, Statistics.RxBytes
    call    Add32

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
    push    ax
    push    bx
    push    cx
    push    dx
    push    ds
    push    es
    push    si
    push    di

    ; Check if receive handler is registered
    cmp     RxHandlerOff, 0
    jne     HasHandler
    cmp     RxHandlerSeg, 0
    je      NoRxHandler

HasHandler:
    ; Set up for LSL call
    ; LSL expects:
    ; ES:SI = packet data
    ; CX = packet length
    ; AX = protocol ID (for us, always 0 - let LSL demux)
    
    mov     ax, cs              ; .COM file: CS=DS=ES
    mov     es, ax
    lea     si, FrameBuffer+3       ; Skip seq + length header
    mov     cx, RxFrameLen
    xor     ax, ax                  ; Protocol ID = 0
    
    ; Call LSL receive handler
    ; The handler will allocate ECB and copy data
    call    dword ptr cs:[RxHandlerOff]
    
NoRxHandler:
    pop     di
    pop     si
    pop     es
    pop     ds
    pop     dx
    pop     cx
    pop     bx
    pop     ax
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
    ; ES:SI = ECB to send
    ; Called by IPX to queue a packet for transmission
    push    ax
    push    bx
    push    cx
    push    dx
    push    ds
    push    es
    push    si
    push    di

    ; Set up data segment
    mov     ax, cs              ; .COM file: CS=DS=ES
    mov     ds, ax

    ; Mark ECB as in use
    mov     byte ptr es:[si+ECB_InUse], 0FFh

    ; Add ECB to transmit queue
    call    QueueTransmitECB

    ; Start transmission if not already transmitting
    cmp     TxPending, 0
    jne     TxAlreadyActive
    call    StartNextTransmit

TxAlreadyActive:
    pop     di
    pop     si
    pop     es
    pop     ds
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    retf
MLIDTransmitHandler ENDP

;=============================================================================
; QueueTransmitECB - Add ECB to transmit queue
; Input: ES:SI = ECB address
;=============================================================================

QueueTransmitECB PROC
    push    ax
    push    bx

    ; Clear the link field (this will be last in queue)
    mov     word ptr es:[si+ECB_Link], 0
    mov     word ptr es:[si+ECB_Link+2], 0

    ; Check if queue is empty
    cmp     word ptr TxQueueHead, 0
    jne     QueueNotEmpty
    cmp     word ptr TxQueueHead+2, 0
    jne     QueueNotEmpty

    ; Queue is empty, this is first ECB
    mov     word ptr TxQueueHead, si
    mov     word ptr TxQueueHead+2, es
    mov     word ptr TxQueueTail, si
    mov     word ptr TxQueueTail+2, es
    mov     TxQueueCount, 1
    jmp     QueueDone

QueueNotEmpty:
    ; Add to tail of queue
    push    es
    push    si
    
    ; Load tail ECB
    les     si, dword ptr TxQueueTail
    
    ; Get new ECB from stack
    pop     bx                      ; New offset
    pop     ax                      ; New segment
    
    ; Link tail to new ECB
    mov     word ptr es:[si+ECB_Link], bx
    mov     word ptr es:[si+ECB_Link+2], ax
    
    ; Update tail pointer
    mov     word ptr TxQueueTail, bx
    mov     word ptr TxQueueTail+2, ax
    inc     TxQueueCount

QueueDone:
    pop     bx
    pop     ax
    ret
QueueTransmitECB ENDP

;=============================================================================
; StartNextTransmit - Begin transmitting next queued ECB
;=============================================================================

StartNextTransmit PROC
    push    ax
    push    bx
    push    cx
    push    dx
    push    es
    push    si

    ; Check if queue is empty
    cmp     word ptr TxQueueHead, 0
    jne     HasPacket
    cmp     word ptr TxQueueHead+2, 0
    je      NoPacket

HasPacket:
    ; Load ECB from queue head
    les     si, dword ptr TxQueueHead
    
    ; Save current ECB
    mov     word ptr TxCurrentECB, si
    mov     word ptr TxCurrentECB+2, es
    mov     TxCurrentSeg, es
    mov     TxCurrentOff, si

    ; Build packet frame from ECB fragments
    call    BuildTransmitFrame
    jc      TransmitError

    ; Start transmitting the frame
    call    TransmitFrame

    ; Mark as transmitting
    mov     TxPending, 1
    mov     TxRetryCount, 0

    jmp     StartDone

TransmitError:
    ; Complete ECB with error
    les     si, dword ptr TxCurrentECB
    mov     byte ptr es:[si+ECB_CompCode], ECB_HARDWARE
    call    CompleteTransmitECB
    
    ; Try next packet
    call    StartNextTransmit

NoPacket:
StartDone:
    pop     si
    pop     es
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret
StartNextTransmit ENDP

;=============================================================================
; BuildTransmitFrame - Build frame from ECB fragments
; Input: ES:SI = ECB
; Output: TxFrameBuffer contains frame, TxPacketSize = size, CF=1 on error
;=============================================================================

BuildTransmitFrame PROC
    push    ax
    push    bx
    push    cx
    push    dx
    push    ds
    push    es
    push    si
    push    di

    ; Set up destination (TxFrameBuffer)
    mov     ax, cs              ; .COM file: CS=DS=ES
    mov     ds, ax
    lea     di, TxFrameBuffer
    
    ; Add frame header: sequence number + length (filled later)
    mov     al, TxSeqNum
    stosb
    inc     TxSeqNum
    
    ; Save position for length field
    push    di
    add     di, 2                   ; Reserve 2 bytes for length
    
    ; Get fragment count
    mov     cx, es:[si+ECB_FragCount]
    test    cx, cx
    jz      BuildError
    
    ; Point to fragment list
    add     si, ECB_FragList
    
    xor     bx, bx                  ; Total size counter
    
BuildFragLoop:
    push    cx
    
    ; Load fragment address and size
    mov     ax, word ptr es:[si+FRAG_Addr]
    mov     dx, word ptr es:[si+FRAG_Addr+2]
    mov     cx, es:[si+FRAG_Size]
    
    ; Check for reasonable size
    test    cx, cx
    jz      BuildFragNext
    cmp     cx, MTU_SIZE
    ja      BuildError
    
    ; Add to total
    add     bx, cx
    cmp     bx, MTU_SIZE
    ja      BuildError
    
    ; Copy fragment data
    push    es
    push    si
    
    mov     si, ax
    mov     es, dx
    
    ; ES:SI = source, DS:DI = dest, CX = count
    push    ds
    pop     ax
    mov     ds, dx
    push    ax
    pop     es
    
    rep     movsb
    
    ; Restore
    pop     si
    pop     es
    
    mov     ax, cs              ; .COM file: CS=DS=ES
    mov     ds, ax
    
BuildFragNext:
    add     si, 6                   ; Next fragment descriptor
    pop     cx
    loop    BuildFragLoop
    
    ; Store total packet size
    mov     TxPacketSize, bx
    
    ; Fill in length field
    pop     di                      ; Position after seq num
    mov     ax, cs              ; .COM file: CS=DS=ES
    mov     es, ax
    mov     ax, bx
    stosw                           ; Store length
    
    clc
    jmp     BuildDone
    
BuildError:
    pop     di                      ; Clean up stack
    stc
    
BuildDone:
    pop     di
    pop     si
    pop     es
    pop     ds
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret
BuildTransmitFrame ENDP

;=============================================================================
; TransmitFrame - Send frame with escaping and CRC
; Input: TxFrameBuffer contains frame, TxPacketSize = size
;=============================================================================

TransmitFrame PROC
    push    ax
    push    bx
    push    cx
    push    dx
    push    si

    ; Calculate CRC for frame
    lea     si, TxFrameBuffer
    mov     cx, TxPacketSize
    add     cx, 3                   ; Include seq + length
    call    CalculateFrameCRC
    
    ; Send frame start delimiter
    mov     al, FRAME_START
    call    SendByte
    
    ; Send frame with escaping
    lea     si, TxFrameBuffer
    mov     cx, TxPacketSize
    add     cx, 3                   ; Include seq + length
    
TransmitLoop:
    lodsb
    call    SendByteWithEscape
    loop    TransmitLoop
    
    ; Send CRC
    mov     ax, word ptr TxCRC
    call    SendByteWithEscape      ; Send low byte
    mov     al, ah
    call    SendByteWithEscape      ; Send high byte
    
    ; Send frame end delimiter
    mov     al, FRAME_START
    call    SendByte
    
    ; Update statistics
    lea     bx, Statistics.TxPackets
    call    Inc32
    mov     ax, TxPacketSize
    call    ZeroExtend16to32    ; DX:AX = 32-bit size
    lea     bx, Statistics.TxBytes
    call    Add32
    
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret
TransmitFrame ENDP

;=============================================================================
; SendByteWithEscape - Send byte with SLIP escaping
; Input: AL = byte to send
;=============================================================================

SendByteWithEscape PROC
    push    ax
    
    cmp     al, FRAME_START
    je      EscapeStart
    cmp     al, FRAME_ESC
    je      EscapeESC
    
    ; Check flow control mode for XON/XOFF escaping
    cmp     CfgData.FlowCtrl, 1
    jne     SendNormal
    
    cmp     al, XON_CHAR
    je      EscapeXON
    cmp     al, XOFF_CHAR
    je      EscapeXOFF
    
SendNormal:
    call    SendByte
    jmp     SendEscDone
    
EscapeStart:
    mov     al, FRAME_ESC
    call    SendByte
    mov     al, ESC_START
    call    SendByte
    jmp     SendEscDone
    
EscapeESC:
    mov     al, FRAME_ESC
    call    SendByte
    mov     al, ESC_ESC
    call    SendByte
    jmp     SendEscDone
    
EscapeXON:
    mov     al, FRAME_ESC
    call    SendByte
    mov     al, ESC_XON
    call    SendByte
    jmp     SendEscDone
    
EscapeXOFF:
    mov     al, FRAME_ESC
    call    SendByte
    mov     al, ESC_XOFF
    call    SendByte
    
SendEscDone:
    pop     ax
    ret
SendByteWithEscape ENDP

;=============================================================================
; SendByte - Send single byte to serial port
; Input: AL = byte
;=============================================================================

SendByte PROC
    push    ax
    push    dx
    
    mov     ah, al
    mov     dx, CfgData.Port
    
    ; Wait for transmitter ready
SendWait:
    push    dx
    add     dx, UART_LSR
    in      al, dx
    test    al, 20h
    pop     dx
    jz      SendWait
    
    ; Send byte
    mov     al, ah
    out     dx, al
    
    pop     dx
    pop     ax
    ret
SendByte ENDP

;=============================================================================
; CalculateFrameCRC - Calculate CRC16 for frame
; Input: DS:SI = data, CX = length
; Output: TxCRC = calculated CRC
;=============================================================================

CalculateFrameCRC PROC
    push    ax
    push    bx
    push    cx
    push    si
    
    mov     word ptr TxCRC, 0FFFFh
    
CalcCRCLoop:
    lodsb
    call    UpdateTxCRC
    loop    CalcCRCLoop
    
    pop     si
    pop     cx
    pop     bx
    pop     ax
    ret
CalculateFrameCRC ENDP

;=============================================================================
; UpdateTxCRC - Update transmit CRC with one byte
; Input: AL = byte
;=============================================================================

UpdateTxCRC PROC
    push    ax
    push    bx
    push    cx
    
    xor     ah, ah
    mov     bx, word ptr TxCRC
    xor     bl, al
    xor     bh, bh
    shl     bx, 1
    mov     cx, word ptr cs:CRCTable[bx]
    mov     ax, word ptr TxCRC
    mov     al, ah              ; shr ax, 8 - move high byte to low
    xor     ah, ah
    xor     ax, cx
    mov     word ptr TxCRC, ax
    
    pop     cx
    pop     bx
    pop     ax
    ret
UpdateTxCRC ENDP

;=============================================================================
; CompleteTransmitECB - Complete ECB and remove from queue
; Input: ES:SI = ECB, CompCode already set
;=============================================================================

CompleteTransmitECB PROC
    push    ax
    push    bx
    push    cx
    push    dx
    push    ds
    push    es
    push    si
    push    di
    
    mov     ax, cs              ; .COM file: CS=DS=ES
    mov     ds, ax
    
    ; Remove from queue head
    mov     ax, word ptr es:[si+ECB_Link]
    mov     dx, word ptr es:[si+ECB_Link+2]
    mov     word ptr TxQueueHead, ax
    mov     word ptr TxQueueHead+2, dx
    
    ; Update count
    dec     TxQueueCount
    jnz     QueueStillHasItems
    
    ; Queue is now empty, clear tail too
    mov     word ptr TxQueueTail, 0
    mov     word ptr TxQueueTail+2, 0
    
QueueStillHasItems:
    ; Clear ECB in use flag
    mov     byte ptr es:[si+ECB_InUse], 0
    
    ; Check if there's an ESR (Event Service Routine)
    cmp     word ptr es:[si+ECB_ESR], 0
    jne     HasESR
    cmp     word ptr es:[si+ECB_ESR+2], 0
    je      NoESR
    
HasESR:
    ; Call the ESR
    ; ES:SI already points to ECB
    call    dword ptr es:[si+ECB_ESR]
    
NoESR:
    ; Clear current ECB
    mov     word ptr TxCurrentECB, 0
    mov     word ptr TxCurrentECB+2, 0
    mov     TxPending, 0
    
    pop     di
    pop     si
    pop     es
    pop     ds
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret
CompleteTransmitECB ENDP

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
    mov     ax, cs              ; .COM file: CS=DS=ES
    mov     ds, ax
    lea     si, Statistics
    mov     cx, SIZE Stats
    rep     movsb
    pop     si
    pop     ds
    retf

DoSendPacket:
    ; ES:SI = ECB to send
    ; This is called directly by control handler
    ; Queue the ECB
    push    ax
    push    bx
    push    ds
    
    mov     ax, cs              ; .COM file: CS=DS=ES
    mov     ds, ax
    
    ; Mark ECB as in use
    mov     byte ptr es:[si+ECB_InUse], 0FFh
    
    ; Queue it
    call    QueueTransmitECB
    
    ; Start transmission if idle
    cmp     TxPending, 0
    jne     SendPktBusy
    call    StartNextTransmit
    
SendPktBusy:
    pop     ds
    pop     bx
    pop     ax
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
    mov     ax, word ptr CfgData.BaudRate
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

    ; Print flow control setting
    cmp     CfgData.FlowCtrl, 0
    je      PrintFlowRTS
    lea     dx, MsgFlowXON
    jmp     PrintFlowStr
PrintFlowRTS:
    lea     dx, MsgFlowRTS
PrintFlowStr:
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
; PrintDetectionInfo - Print port/mode before hardware detection
;=============================================================================

PrintDetectionInfo PROC
    push    ax
    push    dx

    ; Print "Detecting hardware at port "
    lea     dx, MsgDbgPort
    mov     ah, 09h
    int     21h

    ; Print port value in hex
    mov     ax, CfgData.Port
    call    PrintHexWord

    ; Print " mode="
    lea     dx, MsgDbgMode
    mov     ah, 09h
    int     21h

    ; Print mode name
    cmp     CfgData.Mode, 0
    je      PrintModeSerial
    cmp     CfgData.Mode, 1
    je      PrintModePar4
    cmp     CfgData.Mode, 2
    je      PrintModePar8
    jmp     PrintModeEnd

PrintModeSerial:
    lea     dx, MsgDbgSerial
    jmp     PrintModeName
PrintModePar4:
    lea     dx, MsgDbgPar4
    jmp     PrintModeName
PrintModePar8:
    lea     dx, MsgDbgPar8

PrintModeName:
    mov     ah, 09h
    int     21h

PrintModeEnd:
    ; Print newline
    lea     dx, MsgCRLF
    mov     ah, 09h
    int     21h

    pop     dx
    pop     ax
    ret
PrintDetectionInfo ENDP

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
    push    cx
    mov     cl, 4
    shr     al, cl
    call    PrintHexDigit
    pop     cx
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
    ; Check if driver is loaded via multiplex (subfunction 01h to get info)
    mov     ax, MULTIPLEX_ID + 01h  ; AH=ID, AL=01h (get info)
    int     2Fh
    cmp     al, 0FFh
    jne     NotLoaded
    
    ; ES = TSR segment, BX = board number
    mov     Unload_TSRSeg, es
    mov     Unload_TSRBoardNum, bx
    
    ; Check if driver is busy (access TSR's data via ES)
    cmp     byte ptr es:[DriverActive], 1
    je      DriverBusy
    
    ; Check if transmit pending
    cmp     byte ptr es:[TxPending], 1
    je      DriverBusy
    
    ; Save TSR config data for later use
    mov     ax, es:[CfgData.Port]
    mov     Unload_TSRPort, ax
    mov     al, es:[CfgData.Mode]
    mov     Unload_TSRMode, al
    mov     al, es:[CfgData.IRQNum]
    mov     Unload_TSRIRQNum, al
    
    ; ===== CHECK IF WE'RE THE LAST INT 2FH HANDLER =====
    ; Get current INT 2Fh vector
    mov     ax, 352Fh
    int     21h                     ; ES:BX = current INT 2Fh handler
    
    ; Compare with TSR's Int2FHandler address
    mov     ax, Unload_TSRSeg
    cmp     bx, OFFSET Int2FHandler ; Check offset
    jne     NotLastHandler
    mov     cx, es
    cmp     cx, ax                  ; Check segment
    jne     NotLastHandler
    
    ; Restore ES to TSR segment for further use
    mov     es, Unload_TSRSeg
    
    ; ===== DEREGISTER FROM LSL =====
    ; Call DeregisterMLID via LSL MLID Support entry point
    ; The entry point is in the TSR's LSLInfoBuf.LSLEntryPoint
    ; BX = 0006h (DeregisterMLID = MLIDSUP_DEREGISTER_MLID), AX = board number
    ; MUST use ES: override since DS points to unloader, not TSR!
    mov     ax, es:[LSLBoardNum]    ; Get board number from TSR
    mov     bx, 0006h               ; DeregisterMLID function (6, not 2!)
    call    dword ptr es:[LSLInfoBuf.LSLEntryPoint]
    
    ; ===== CALL MLID SHUTDOWN (PERMANENT) =====
    ; Call the TSR's MLIDShutdownProc to clear internal state
    ; CX=0000h indicates permanent shutdown (for cleanup only, already deregistered)
    mov     es, Unload_TSRSeg       ; Restore ES in case LSL call modified it
    xor     cx, cx                  ; CX=0000h = permanent shutdown
    call    dword ptr es:[MLIDFunctions.MLIDShutdown]
    
    ; ===== DISABLE HARDWARE INTERRUPTS =====
    mov     dx, Unload_TSRPort
    cmp     Unload_TSRMode, 0
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
    ; ===== DISABLE IRQ AT PIC =====
    mov     al, Unload_TSRIRQNum
    cmp     al, 8
    jb      DisablePIC1
    
    ; IRQ 8-15: Use slave PIC (port 0A1h)
    sub     al, 8
    mov     cl, al
    mov     al, 1
    shl     al, cl                  ; AL = mask bit for this IRQ
    mov     ah, al                  ; Save mask bit
    in      al, 0A1h
    or      al, ah                  ; Set the mask bit to disable IRQ
    out     0A1h, al
    jmp     IRQDisabled

DisablePIC1:
    ; IRQ 0-7: Use master PIC (port 021h)
    mov     cl, al
    mov     al, 1
    shl     al, cl                  ; AL = mask bit for this IRQ
    mov     ah, al                  ; Save mask bit
    in      al, 021h
    or      al, ah                  ; Set the mask bit to disable IRQ
    out     021h, al

IRQDisabled:
    ; ===== RESTORE INTERRUPT VECTORS =====
    ; Access TSR's saved vectors via Unload_TSRSeg
    mov     es, Unload_TSRSeg
    
    ; Restore INT 2Fh vector
    mov     dx, es:[OldInt2F_Off]
    mov     ax, es:[OldInt2F_Seg]
    push    ds
    mov     ds, ax
    mov     ax, 252Fh
    int     21h
    pop     ds

    ; Restore hardware interrupt vector
    ; First get the old vector from TSR
    mov     es, Unload_TSRSeg
    mov     dx, es:[OldHwInt_Off]
    mov     bx, es:[OldHwInt_Seg]   ; BX = segment
    
    ; Calculate interrupt number
    mov     al, Unload_TSRIRQNum
    cmp     al, 8
    jb      RestoreBelow8
    sub     al, 8
    add     al, 70h
    jmp     DoRestore

RestoreBelow8:
    add     al, 08h

DoRestore:
    ; AL = interrupt number, BX:DX = old vector
    push    ds
    mov     ds, bx                  ; DS = old segment
    mov     ah, 25h
    int     21h
    pop     ds

    ; ===== FREE TSR MEMORY =====
    mov     es, Unload_TSRSeg
    mov     ah, 49h                 ; Free memory block
    int     21h

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

NotLastHandler:
    lea     dx, MsgErrNotLast
    mov     ah, 09h
    int     21h
    mov     ax, 4C01h
    int     21h
UnloadDriver ENDP

;=============================================================================
; 32-bit Arithmetic Helper Procedures for 8086 Compatibility
;=============================================================================

; Inc32 - Increment a DWORD memory operand
; Input: BX = offset of DWORD to increment
; Destroys: AX, BX
Inc32 PROC
    add     word ptr [bx], 1    ; Increment low word
    adc     word ptr [bx+2], 0  ; Add carry to high word
    ret
Inc32 ENDP

; Add32 - Add DX:AX to a DWORD memory operand
; Input: DX:AX = 32-bit value to add
;        BX = offset of DWORD destination
; Destroys: none (all registers preserved)
Add32 PROC
    push    ax
    push    dx
    add     word ptr [bx], ax   ; Add low word
    adc     word ptr [bx+2], dx ; Add high word with carry
    pop     dx
    pop     ax
    ret
Add32 ENDP

; ZeroExtend16to32 - Zero-extend AX to DX:AX (32-bit)
; Input: AX = 16-bit value
; Output: DX:AX = 32-bit zero-extended value
; Destroys: DX
ZeroExtend16to32 PROC
    xor     dx, dx              ; Clear high word
    ret
ZeroExtend16to32 ENDP

; LookupBaudDivisor - Find divisor for configured baud rate
; Input: CfgData.BaudRate (DWORD)
; Output: BX = divisor value
; Destroys: AX, CX, SI
LookupBaudDivisor PROC
    push    di
    lea     si, BaudTable
    
BaudLoop:
    ; Load baud rate from table (DWORD)
    mov     ax, word ptr [si]
    mov     cx, word ptr [si+2]
    
    ; Check for end marker (baud rate = 0)
    or      ax, cx
    jz      BaudNotFound
    
    ; Compare with configured baud rate
    mov     ax, word ptr [si]
    mov     cx, word ptr [si+2]
    cmp     ax, word ptr CfgData.BaudRate
    jne     BaudNext
    cmp     cx, word ptr CfgData.BaudRate+2
    jne     BaudNext
    
    ; Found match - get divisor
    mov     bx, word ptr [si+4]
    pop     di
    ret
    
BaudNext:
    add     si, 6               ; Move to next entry (4 bytes baud + 2 bytes divisor)
    jmp     BaudLoop
    
BaudNotFound:
    ; Default to 38400 baud if not found (divisor = 3)
    mov     bx, 3
    pop     di
    ret
LookupBaudDivisor ENDP

;=============================================================================
; End of Resident Code
;=============================================================================

ResidentEnd:

; Calculate resident size
ResidentSize EQU (ResidentEnd - Start)

CODE    ENDS

        END Start

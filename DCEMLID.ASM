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

; Parallel Port Status Bits
LPT_STAT_NIRQ   EQU 04h                 ; nIRQ (active low) - interrupt pending
LPT_STAT_ERROR  EQU 08h                 ; nError (active low)
LPT_STAT_ONLINE EQU 10h                 ; Select/Online
LPT_STAT_PAPER  EQU 20h                 ; Paper Out
LPT_STAT_NACK   EQU 40h                 ; nAck (active low) - data acknowledged  
LPT_STAT_NBUSY  EQU 80h                 ; nBusy (active low) - peripheral busy

; Buffer Sizes
RX_BUFFER_SIZE  EQU 4096                ; 4KB receive circular buffer
TX_BUFFER_SIZE  EQU 2048                ; 2KB transmit queue

; Timing Constants
RETRY_TIMEOUT   EQU 200                 ; 200ms timeout
MAX_RETRIES     EQU 2                   ; Maximum retry attempts
MAX_CONFIG_LEN  EQU 512                 ; Maximum config string length

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
ECB_TIMEOUT     EQU 0FAh                ; Timeout waiting for ACK
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
    IRQNum      DB 0FFh                 ; IRQ number (FFh = auto-detect)
    UARTType    DB 0                    ; 0=8250, 1=16550
    ParMode     DB 0                    ; For PAR8: 0=EPP, 1=PS/2
    BoardNum    DW 0                    ; Board number for multiple instances
    PortSource  DB 1                    ; 0=BDA lookup, 1=Raw address
    FrameType   DW 0005h                ; Frame type ID (default: 802.2)
Config ENDS

CfgData Config <>

; ODI Statistics Structure (per ODI Spec v4.00 page 15-18)
MLIDStatStructure STRUC
    MStatTableMajorVer    DB 1            ; 00h: Major version (1)
    MStatTableMinorVer    DB 1            ; 01h: Minor version (01) = v1.01
    MNumGenericCounters   DW 14           ; 02h: Number of generic counters
    MValidCounterMask     DD 0            ; 04h: Which counters are valid (0=all valid)
    MTotalTxPackets       DD 0            ; 08h: Total TX packets
    MTotalRxPackets       DD 0            ; 0Ch: Total RX packets
    MNoECBsAvailable      DD 0            ; 10h: No ECBs available count
    MTxTooBig             DD 0            ; 14h: TX too big count
    MTxTooSmall           DD 0            ; 18h: TX too small count
    MRxOverflow           DD 0            ; 1Ch: RX overflow count
    MRxTooBig             DD 0            ; 20h: RX too big count
    MRxTooSmall           DD 0            ; 24h: RX too small count
    MTxMiscError          DD 0            ; 28h: TX misc error count
    MRxMiscError          DD 0            ; 2Ch: RX misc error count
    MTxRetryCount         DD 0            ; 30h: TX retry count
    MRxChecksumError      DD 0            ; 34h: RX checksum error count
    MRxMismatchError      DD 0            ; 38h: RX mismatch error count
    MQueueDepth           DD 0            ; 3Ch: Queue depth
    MTxOKMulticastCount   DD 0            ; 40h: TX multicast count
    MRxOKMulticastCount   DD 0            ; 44h: RX multicast count
    MNumCustomCounters    DW 0            ; 48h: Number of custom counters (0)
MLIDStatStructure ENDS

MLIDStatistics MLIDStatStructure <>

; Legacy Stats alias (for backward compat with existing code)
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

; Transmit Monitor (for RegisterTxMonitor BX=4)
TxMonitorOff    DW 0                    ; TX monitor routine offset
TxMonitorSeg    DW 0                    ; TX monitor routine segment
TxMonitorActive DB 0                    ; TX monitor enabled flag

; Current LookAhead Size (for SetLookAheadSize BX=9)
CurrentLookAhead DW 64                  ; Default 64 bytes

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
TxPaused        DB 0                    ; TX paused by XON/XOFF flow control
TxRetransmit    DB 0                    ; NAK received - retransmit needed
TxRetryCount    DB 0                    ; Current retry count
TxTimeout       DW 0                    ; Timeout counter
TxInPtr         DW 0                    ; TX buffer write pointer
TxOutPtr        DW 0                    ; TX buffer read pointer

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

; Saved ECB pointer for receive processing
SavedECBOff     DW 0                    ; Saved ECB offset during copy
SavedECBSeg     DW 0                    ; Saved ECB segment during copy

; Receive State
RxSeqNum        DB 0                    ; Expected RX sequence number
RxState         DB 0                    ; Frame parser FSM state
RxEscaped       DB 0                    ; Escape sequence flag
RxBufIdx        DW 0                    ; Current buffer index
RxFrameLen      DW 0                    ; Current frame length
RxCRC           DW 0FFFFh               ; Running CRC calculation

; Backpressure State
RxPaused        DB 0                    ; 1 = we signaled pause to remote
RxPauseThresh   DW 3584                 ; Pause threshold (RX_BUFFER_SIZE - 512)
RxResumeThresh  DW 1024                 ; Resume threshold (1KB free)

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
CfgKeyFrame     DB 'FRAME',0
CfgValSerial    DB 'SERIAL',0
CfgValPar4      DB 'PAR4',0
CfgValPar8      DB 'PAR8',0
CfgValRTS       DB 'RTS',0
CfgValXON       DB 'XON',0

; Frame Type Values
CfgVal8022      DB 'ETHERNET_802.2',0
CfgVal8023      DB 'ETHERNET_802.3',0
CfgValEthII     DB 'ETHERNET_II',0
CfgValSNAP      DB 'ETHERNET_SNAP',0

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
MsgBaud         DB ' Baud='
MsgBaud9600     DB '9600$'
MsgBaud19200    DB '19200$'
MsgBaud38400    DB '38400$'
MsgBaud57600    DB '57600$'
MsgBaud115200   DB '115200$'
MsgBaudUnknown  DB '?$'
Msg8250         DB ' (8250)$'
Msg16550        DB ' (16550)$'
MsgFlowRTS      DB ' Flow=RTS/CTS$'
MsgFlowXON      DB ' Flow=XON/XOFF$'
MsgUMB          DB ' [UMB]$'
MsgConv         DB ' [Conventional]$'
MsgCRLF         DB 13,10,'$'

; Verbose mode statistics messages
MsgStatsTX      DB 'TX:$'
MsgStatsRX      DB ' RX:$'
MsgStatsErr     DB ' Err:$'

MsgWarn8250     DB 'WARNING: 8250 UART at high baud - packet loss may occur.',13,10
                DB '         Consider using 38400 or lower.',13,10,'$'

MsgWarnCOM34    DB 'WARNING: COM3/COM4 IRQ not specified in NET.CFG.',13,10
                DB '         Using default (may conflict). Add INT to config.',13,10,'$'

MsgWarnDeregister DB 'WARNING: LSL deregistration failed. Driver may be partially unloaded.',13,10,'$'

MsgErrLSL       DB 'ERROR: LSL not loaded. Load LSL.COM first.',13,10,'$'
MsgErrConfig    DB 'ERROR: No configuration in NET.CFG. Add Link Driver DCEMLID section.',13,10,'$'
MsgErrLoaded    DB 'ERROR: Driver already loaded.',13,10,'$'
MsgErrIRQ       DB 'ERROR: IRQ conflict detected.',13,10,'$'
MsgErrPort      DB 'ERROR: Hardware not detected at specified port.',13,10,'$'
MsgErrBDAPort   DB 'ERROR: Port not present in BIOS data area.',13,10,'$'
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
                DB 13,10,'Usage: DCEMLID [/D] [/V] [/U] [/?]',13,10
                DB 13,10,'  /D  Enable debug output during hardware detection',13,10
                DB '  /V  Verbose mode - show live statistics',13,10
                DB '  /U  Unload driver from memory',13,10
                DB '  /?  Show this help',13,10
                DB 13,10,'Configuration is read from NET.CFG. Example:',13,10
                DB 13,10,'  Link Driver DCEMLID',13,10
                DB '     PORT COM1           ; or COM2, COM3, COM4 (from BIOS)',13,10
                DB '     PORT LPT1           ; or LPT2, LPT3 (from BIOS)',13,10
                DB '     PORT 3F8            ; raw hex address (manual detection)',13,10
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
    cmp     al, 'V'
    je      SetVerboseFlag
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
    
SetVerboseFlag:
    mov     CmdVerbose, 1
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
CmdVerbose      DB 0

; Timer hook for verbose mode statistics
OldInt1C_Off    DW 0
OldInt1C_Seg    DW 0
VerboseTicks    DW 0                ; Counter for tick-based stats display
VERBOSE_INTERVAL EQU 18 * 5         ; ~5 seconds (18 ticks/sec)

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
    
    ; Initialize length limit counter
    mov     cx, MAX_CONFIG_LEN      ; Maximum config string length

ParseLSLLoop:
    ; Check length limit to prevent runaway parsing
    dec     cx
    jz      ParseLSLDone            ; Limit reached - stop parsing
    
    ; Check for end of configuration
    mov     al, es:[si]
    test    al, al
    jz      ParseLSLDone
    
    ; Skip spaces
SkipLSLSpaces:
    cmp     byte ptr es:[si], ' '
    jne     NotSpace
    inc     si
    dec     cx                      ; Count skipped spaces
    jz      ParseLSLDone
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
    
    ; Check for FRAME keyword
    lea     di, CfgKeyFrame
    call    CompareLSLKeyword
    je      ParseLSLFrame
    
    ; Unknown keyword, skip to end of line
    call    SkipToLSLEOL
    jmp     ParseLSLLoop

ParseLSLPort:
    ; === Check for COMn / LPTn symbolic names ===
    ; Read first char
    mov     al, es:[si]
    call    ToUpper
    
    cmp     al, 'C'
    je      TryParseCOM
    cmp     al, 'L'
    je      TryParseLPT
    jmp     ParsePortHex        ; Not COM or LPT, try hex

TryParseCOM:
    ; Check for "OM" after 'C'
    mov     al, es:[si+1]
    call    ToUpper
    cmp     al, 'O'
    jne     ParsePortHex
    mov     al, es:[si+2]
    call    ToUpper
    cmp     al, 'M'
    jne     ParsePortHex
    
    ; Get port number (1-4)
    mov     al, es:[si+3]
    cmp     al, '1'
    jb      ParsePortHex
    cmp     al, '4'
    ja      ParsePortHex
    
    ; Valid COM1-4, advance SI past "COMn"
    add     si, 4
    sub     al, '1'             ; Convert '1'-'4' to 0-3
    call    LookupBDACOM
    jmp     ParseLSLLoop

TryParseLPT:
    ; Check for "PT" after 'L'
    mov     al, es:[si+1]
    call    ToUpper
    cmp     al, 'P'
    jne     ParsePortHex
    mov     al, es:[si+2]
    call    ToUpper
    cmp     al, 'T'
    jne     ParsePortHex
    
    ; Get port number (1-3)
    mov     al, es:[si+3]
    cmp     al, '1'
    jb      ParsePortHex
    cmp     al, '3'
    ja      ParsePortHex
    
    ; Valid LPT1-3, advance SI past "LPTn"
    add     si, 4
    sub     al, '1'             ; Convert '1'-'3' to 0-2
    call    LookupBDALPT
    jmp     ParseLSLLoop

ParsePortHex:
    ; Not a symbolic name, parse as hex address (raw)
    call    ParseLSLHexValue
    jc      ParseLSLLoop
    mov     CfgData.Port, ax
    mov     CfgData.PortSource, 1       ; Raw address
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

ParseLSLFrame:
    ; Parse FRAME type value
    lea     di, CfgVal8022
    call    CompareLSLValue
    je      SetFrame8022
    
    lea     di, CfgVal8023
    call    CompareLSLValue
    je      SetFrame8023
    
    lea     di, CfgValEthII
    call    CompareLSLValue
    je      SetFrameEthII
    
    lea     di, CfgValSNAP
    call    CompareLSLValue
    je      SetFrameSNAP
    
    ; Unknown frame type - ignore
    jmp     ParseLSLLoop

SetFrame8022:
    mov     CfgData.FrameType, 0005h    ; Ethernet 802.2
    jmp     ParseLSLLoop

SetFrame8023:
    mov     CfgData.FrameType, 0002h    ; Ethernet 802.3
    jmp     ParseLSLLoop

SetFrameEthII:
    mov     CfgData.FrameType, 0004h    ; Ethernet II
    jmp     ParseLSLLoop

SetFrameSNAP:
    mov     CfgData.FrameType, 000Ah    ; Ethernet SNAP
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
; LookupBDACOM - Look up COM port address from BIOS Data Area
; Input: AL = port index (0=COM1, 1=COM2, 2=COM3, 3=COM4)
; Output: CfgData.Port set, CfgData.PortSource=0 if found
;         CF=1 if port not present in BDA
;=============================================================================

LookupBDACOM PROC
    push    es
    push    bx
    
    ; BDA segment
    xor     bx, bx
    mov     es, bx
    
    ; COM ports at 0040:0000 (2 bytes each)
    xor     ah, ah
    shl     al, 1               ; Multiply index by 2
    mov     bx, 0400h           ; BDA COM port base
    add     bx, ax
    
    mov     ax, es:[bx]         ; Get port address
    test    ax, ax
    jz      BDACOMNotFound
    
    mov     CfgData.Port, ax
    mov     CfgData.PortSource, 0   ; From BDA
    clc
    pop     bx
    pop     es
    ret

BDACOMNotFound:
    ; Port not present, print error
    push    dx
    lea     dx, MsgErrBDAPort
    mov     ah, 09h
    int     21h
    pop     dx
    stc
    pop     bx
    pop     es
    ret
LookupBDACOM ENDP

;=============================================================================
; LookupBDALPT - Look up LPT port address from BIOS Data Area
; Input: AL = port index (0=LPT1, 1=LPT2, 2=LPT3)
; Output: CfgData.Port set, CfgData.PortSource=0 if found
;         CF=1 if port not present in BDA
;=============================================================================

LookupBDALPT PROC
    push    es
    push    bx
    
    ; BDA segment
    xor     bx, bx
    mov     es, bx
    
    ; LPT ports at 0040:0008 (2 bytes each)
    xor     ah, ah
    shl     al, 1               ; Multiply index by 2
    mov     bx, 0408h           ; BDA LPT port base
    add     bx, ax
    
    mov     ax, es:[bx]         ; Get port address
    test    ax, ax
    jz      BDALPTNotFound
    
    mov     CfgData.Port, ax
    mov     CfgData.PortSource, 0   ; From BDA
    clc
    pop     bx
    pop     es
    ret

BDALPTNotFound:
    ; Port not present, print error
    push    dx
    lea     dx, MsgErrBDAPort
    mov     ah, 09h
    int     21h
    pop     dx
    stc
    pop     bx
    pop     es
    ret
LookupBDALPT ENDP

;=============================================================================
; IODelay - Small delay for I/O register settling
; Executes ~4-8 CPU cycles delay using JMP $+2 technique
;=============================================================================

IODelay PROC
    jmp     short $+2           ; 2 cycles
    jmp     short $+2           ; 2 cycles
    jmp     short $+2           ; 2 cycles
    jmp     short $+2           ; 2 cycles
    ret
IODelay ENDP

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

    ; Hook timer interrupt for verbose mode if requested
    cmp     CmdVerbose, 0
    je      SkipVerboseHook
    call    HookTimerInt
SkipVerboseHook:

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
    push    bx

    mov     dx, CfgData.Port

    ; Check if port came from BDA (already validated by BIOS)
    cmp     CfgData.PortSource, 0
    je      UARTDetected        ; Skip detection, go straight to 16550 check

    ; Initialize retry counter for UART detection
    mov     bl, 3               ; 3 retry attempts for slow 8250

UARTDetectRetry:
    ; === Raw address: UART Detection using Line Control Register (LCR) test ===
    ; The original 8250 does NOT have a scratch register (offset 7),
    ; so we use the LCR (offset 3) instead for reliable detection.
    ; Save current LCR, write test patterns, then restore.
    mov     dx, CfgData.Port    ; Reset DX in case of retry
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
    call    IODelay             ; Allow register to settle
    mov     ah, al              ; Store in AH
    
    ; Test pattern 1
    mov     al, 01Bh            ; Test pattern 1 (8N1 + break)
    out     dx, al
    call    IODelay             ; Allow register to settle
    call    IODelay             ; Extra delay for slow hardware
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
    call    IODelay             ; Allow register to settle
    call    IODelay             ; Extra delay for slow hardware
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
    jne     NoUARTRetry
    push    dx
    lea     dx, MsgDbgFAIL
    mov     ah, 09h
    int     21h
    pop     dx
    jmp     NoUARTRetry

NoUARTRetry:
    ; Check if we should retry
    dec     bl                  ; Decrement retry counter
    jz      NoUARTFound         ; No more retries - fail
    
    ; Delay before retry (allow slow 8250 to settle)
    mov     cx, 1000h
NoUARTRetryDelay:
    call    IODelay
    loop    NoUARTRetryDelay
    jmp     UARTDetectRetry     ; Try again

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

    ; Determine IRQ number from port (if not specified by user)
    cmp     CfgData.IRQNum, 0FFh    ; Was INT specified in config?
    jne     SerialIRQSet            ; Yes - use user-specified value
    
    mov     ax, CfgData.Port
    cmp     ax, 3F8h
    je      SerialIRQ4
    cmp     ax, 2F8h
    je      SerialIRQ3
    
    ; COM3/COM4 - warn about non-standard IRQ
    cmp     ax, 3E8h
    je      SerialIRQ4_Warn
    cmp     ax, 2E8h
    je      SerialIRQ3_Warn
    
    ; Unknown port - default to IRQ4
    jmp     SerialIRQ4
    
SerialIRQ4_Warn:
    ; COM3 at 3E8h - warn and use IRQ4
    push    dx
    lea     dx, MsgWarnCOM34
    mov     ah, 09h
    int     21h
    pop     dx
    jmp     SerialIRQ4
    
SerialIRQ3_Warn:
    ; COM4 at 2E8h - warn and use IRQ3
    push    dx
    lea     dx, MsgWarnCOM34
    mov     ah, 09h
    int     21h
    pop     dx
    jmp     SerialIRQ3
    
SerialIRQ4:
    mov     CfgData.IRQNum, 4
    jmp     SerialIRQSet

SerialIRQ3:
    mov     CfgData.IRQNum, 3

SerialIRQSet:
    clc
    pop     bx
    pop     cx
    pop     ax
    pop     dx
    ret

NoUARTFound:
    ; No UART detected at this port
    stc
    pop     bx
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

    ; Check if port came from BDA (already validated by BIOS)
    cmp     CfgData.PortSource, 0
    je      LPTDetected         ; Skip basic detection, go to EPP/mode check

    ; === Raw address: Parallel port detection using data register test ===
    ; Write test patterns and read back with IO delays
    mov     al, 055h
    out     dx, al
    call    IODelay             ; Allow register to settle
    call    IODelay             ; Extra delay for slow hardware
    in      al, dx
    cmp     al, 055h
    jne     NoLPTFound
    
    mov     al, 0AAh
    out     dx, al
    call    IODelay             ; Allow register to settle
    call    IODelay             ; Extra delay for slow hardware
    in      al, dx
    cmp     al, 0AAh
    jne     NoLPTFound

LPTDetected:
    ; Parallel port detected/validated, continue with initialization

    ; Check for PAR8 mode
    cmp     CfgData.Mode, 2
    jne     Par4Mode

    ; Try to detect EPP capability (always do this, BIOS doesn't detect EPP)
    mov     dx, CfgData.Port
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
    shr     cx, 1               ; Divide by 2 for word count
    rep     movsw               ; Copy words
    adc     cx, cx              ; CX=1 if odd byte remains
    rep     movsb               ; Copy trailing byte if any
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
    mov     ch, 1
    shl     ch, cl
    not     ch                  ; CH = mask with IRQ bit cleared
    
    in      al, 0A1h            ; Read PIC2 mask
    and     al, ch              ; Clear the IRQ bit
    out     0A1h, al
    jmp     EnableDone

EnablePIC1:
    mov     cl, al
    mov     ch, 1
    shl     ch, cl
    not     ch                  ; CH = mask with IRQ bit cleared
    
    in      al, 021h            ; Read PIC1 mask
    and     al, ch              ; Clear the IRQ bit
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
; HookTimerInt - Hook INT 1Ch for verbose mode statistics display
;=============================================================================

HookTimerInt PROC
    push    ax
    push    bx
    push    dx
    push    es
    
    ; Get current INT 1Ch vector
    mov     ax, 351Ch
    int     21h
    mov     OldInt1C_Off, bx
    mov     OldInt1C_Seg, es
    
    ; Set new INT 1Ch vector
    mov     dx, offset TimerISR
    mov     ax, 251Ch
    int     21h
    
    ; Initialize tick counter
    mov     VerboseTicks, VERBOSE_INTERVAL
    
    pop     es
    pop     dx
    pop     bx
    pop     ax
    ret
HookTimerInt ENDP

;=============================================================================
; TimerISR - Timer interrupt (INT 1Ch) handler for verbose stats
;=============================================================================

TimerISR PROC FAR
    pushf
    
    ; Decrement tick counter
    dec     cs:VerboseTicks
    jnz     TimerISRChain
    
    ; Reset counter
    mov     cs:VerboseTicks, VERBOSE_INTERVAL
    
    ; Display statistics (save all registers)
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di
    push    ds
    push    es
    
    ; Set DS to our segment
    push    cs
    pop     ds
    
    ; Print stats header (cursor return, overwrite line)
    mov     dl, 0Dh             ; CR only (no LF, overwrite line)
    mov     ah, 02h
    int     21h
    
    ; Print "TX:"
    lea     dx, MsgStatsTX
    mov     ah, 09h
    int     21h
    
    ; Print TX count
    mov     ax, word ptr Statistics.TxPackets
    call    PrintDecimal
    
    ; Print " RX:"
    lea     dx, MsgStatsRX
    mov     ah, 09h
    int     21h
    
    ; Print RX count
    mov     ax, word ptr Statistics.RxPackets
    call    PrintDecimal
    
    ; Print " Err:"
    lea     dx, MsgStatsErr
    mov     ah, 09h
    int     21h
    
    ; Print CRC errors
    mov     ax, word ptr Statistics.CRCErrors
    call    PrintDecimal
    
    ; Print spaces to clear rest of line
    mov     dl, ' '
    mov     ah, 02h
    int     21h
    int     21h
    int     21h
    int     21h
    
    pop     es
    pop     ds
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    
TimerISRChain:
    popf
    
    ; Chain to old handler
    jmp     dword ptr cs:OldInt1C_Off
TimerISR ENDP

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

; LookAheadStruc - Required for packet reception (ODI Spec page 11-14)
; This structure is passed to GetStackECB to obtain an ECB from protocol stack
LookAheadStruc STRUC
    LMediaHeaderPtr         DD ?            ; 00h: Far ptr to media header
    LookAheadPtr            DD ?            ; 04h: Far ptr to lookahead data
    LookAheadLen            DW ?            ; 08h: Length of lookahead data
    LProtID                 DB 6 DUP(?)     ; 0Ah: Protocol ID (6 bytes)
    LBoardNum               DW ?            ; 10h: Logical board number
    LDataSize               DW ?            ; 12h: Total packet data size
    LImmAddress             DB 6 DUP(?)     ; 14h: Immediate (source) address
    LPacketAttrib           DW ?            ; 1Ah: Packet attributes
    LDestType               DW ?            ; 1Ch: Destination type (0=unicast, 1=multi, 2=broad)
    LStartCopyOffset        DW ?            ; 1Eh: Start copy offset (set by protocol)
    LPriorityLevel          DB ?            ; 20h: Priority level
    LRESERVED               DB 3 DUP(?)     ; 21h: Reserved
LookAheadStruc ENDS

; LSL MLID Support function numbers
MLIDSUP_GET_ECB             EQU 0
MLIDSUP_RETURN_ECB          EQU 1
MLIDSUP_DEFRAG_ECB          EQU 2
MLIDSUP_SCHEDULE_AES        EQU 3
MLIDSUP_CANCEL_AES          EQU 4
MLIDSUP_GET_INTERVAL        EQU 5
MLIDSUP_DEREGISTER_MLID     EQU 6
MLIDSUP_HOLD_RECV_EVENT     EQU 7
MLIDSUP_START_CRITICAL      EQU 8
MLIDSUP_END_CRITICAL        EQU 9
MLIDSUP_CRITICAL_STATUS     EQU 0Ah
MLIDSUP_SERVICE_EVENTS      EQU 0Bh
MLIDSUP_SEND_COMPLETE       EQU 0Eh
MLIDSUP_ADD_PID             EQU 0Fh
MLIDSUP_GET_STACK_ECB       EQU 10h
MLIDSUP_CONTROL_FILTER      EQU 11h

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

; LookAhead buffer for packet reception (passed to GetStackECB)
RxLookAhead         LookAheadStruc <>

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
    ; Copy node address (6 bytes = 3 words)
    push    ds
    pop     es
    lea     si, CfgData.MacAddr
    lea     di, MLIDConfigTable.NodeAddress
    mov     cx, 3
    rep     movsw

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
    
    ; Frame type (from config, default: Ethernet_802.2 = 0005h)
    mov     ax, CfgData.FrameType
    mov     MLIDConfigTable.FrameTypeID, ax
    
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

; DCEMLID uses C0A0h/C0A1h, LSL uses C000h-C00Fh
; We must check for our specific AL values to avoid intercepting LSL calls!

Int2FHandler PROC FAR
    cmp     ax, 0C0A0h              ; Installation check (our specific ID)
    je      Mux_InstallCheck
    cmp     ax, 0C0A1h              ; Get info subfunction
    je      Mux_GetInfo
    
    ; Not ours - chain to previous handler
    jmp     dword ptr cs:[OldInt2F_Off]

Mux_InstallCheck:
    ; AL=FFh = installed, ES = TSR segment
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
    push    cx
    push    dx
    push    ds

    mov     ax, cs              ; .COM file: CS=DS=ES
    mov     ds, ax

SerialISR_Loop:
    ; Read IIR to determine interrupt source
    mov     dx, CfgData.Port
    add     dx, UART_IIR
    in      al, dx
    
    ; Bit 0 = 1 means no interrupt pending
    test    al, 01h
    jnz     SerialISR_Done
    
    ; Mask interrupt ID bits (1-2)
    and     al, 06h
    
    ; Check for Line Status interrupt (IIR=06h) - highest priority
    cmp     al, 06h
    je      SerialLineStatus
    
    ; Check for Received Data interrupt (IIR=04h)
    cmp     al, 04h
    je      SerialReceive
    
    ; Check for THRE interrupt (IIR=02h)
    cmp     al, 02h
    je      SerialTHRE
    
    ; Modem status or unknown - ignore
    jmp     SerialISR_Loop

SerialLineStatus:
    ; Read LSR to clear interrupt and check errors
    mov     dx, CfgData.Port
    add     dx, UART_LSR
    in      al, dx
    
    ; Check for errors (bits 1-4)
    test    al, 1Eh             ; OE, PE, FE, BI bits
    jz      SerialISR_Loop      ; No errors
    
    ; Increment error counter based on error type
    test    al, 02h             ; Overrun Error
    jz      CheckParity
    inc     word ptr MLIDStatistics.MRxOverflow
    adc     word ptr MLIDStatistics.MRxOverflow+2, 0
    ; Signal sender to pause on overrun (backpressure)
    call    SendBackpressure
CheckParity:
    test    al, 04h             ; Parity Error
    jz      CheckFraming
    inc     word ptr MLIDStatistics.MRxMiscError
    adc     word ptr MLIDStatistics.MRxMiscError+2, 0
CheckFraming:
    test    al, 08h             ; Framing Error
    jz      CheckBreak
    inc     word ptr MLIDStatistics.MRxMiscError
    adc     word ptr MLIDStatistics.MRxMiscError+2, 0
CheckBreak:
    test    al, 10h             ; Break Indicator
    jz      SerialISR_Loop
    ; Break received - reset frame parser state
    mov     RxState, STATE_IDLE
    jmp     SerialISR_Loop

SerialTHRE:
    ; Transmit Holding Register Empty - send next byte if available
    cmp     TxPending, 0
    je      SerialISR_Loop      ; Nothing to send
    
    ; Check if flow control paused
    cmp     TxPaused, 0
    jne     SerialISR_Loop      ; Paused by XON/XOFF
    
    ; Check if retransmit requested (NAK received)
    cmp     TxRetransmit, 0
    je      SerialNoRetransmit
    mov     TxRetransmit, 0     ; Clear retransmit flag
    mov     TxOutPtr, 0         ; Reset to start of frame
    inc     TxRetryCount
    cmp     TxRetryCount, MAX_RETRIES
    jb      SerialNoRetransmit
    ; Max retries - fail this transmission
    jmp     SerialTxFailed
    
SerialNoRetransmit:
    ; Get next byte from TX buffer
    mov     si, TxOutPtr
    cmp     si, TxInPtr
    je      SerialTxEmpty       ; Buffer empty
    
    mov     al, TxFrameBuffer[si]
    inc     si
    cmp     si, TX_BUFFER_SIZE
    jb      SerialNoWrap
    xor     si, si              ; Wrap around
SerialNoWrap:
    mov     TxOutPtr, si
    
    ; Send the byte
    mov     dx, CfgData.Port
    out     dx, al
    
    ; Check if buffer now empty
    mov     si, TxOutPtr
    cmp     si, TxInPtr
    jne     SerialISR_Loop      ; More data to send
    
SerialTxEmpty:
    ; Buffer empty - disable THRE interrupt
    mov     TxPending, 0
    mov     dx, CfgData.Port
    add     dx, UART_IER
    in      al, dx
    and     al, 0FDh            ; Clear THRE interrupt enable (bit 1)
    out     dx, al
    
    ; Complete current ECB if any
    push    es
    les     si, dword ptr TxCurrentECB
    cmp     si, 0
    je      SerialNoECB
    cmp     word ptr TxCurrentECB+2, 0
    je      SerialNoECB
    mov     byte ptr es:[si+ECB_CompCode], ECB_SUCCESS
    call    CompleteTransmitECB
SerialNoECB:
    pop     es
    jmp     SerialISR_Loop

SerialTxFailed:
    ; Max retries exceeded - fail this transmission
    mov     TxPending, 0
    mov     TxRetryCount, 0     ; Reset for next transmission
    mov     dx, CfgData.Port
    add     dx, UART_IER
    in      al, dx
    and     al, 0FDh            ; Disable THRE interrupt
    out     dx, al
    
    ; Update failure statistics
    inc     word ptr MLIDStatistics.MTxRetryCount
    adc     word ptr MLIDStatistics.MTxRetryCount+2, 0
    
    ; Fail current ECB
    push    es
    les     si, dword ptr TxCurrentECB
    cmp     si, 0
    je      SerialTxFailNoECB
    cmp     word ptr TxCurrentECB+2, 0
    je      SerialTxFailNoECB
    mov     byte ptr es:[si+ECB_CompCode], ECB_HARDWARE
    call    CompleteTransmitECB
SerialTxFailNoECB:
    pop     es
    jmp     SerialISR_Loop

SerialReceive:
    ; Read received data
    mov     dx, CfgData.Port
    in      al, dx
    
    ; Check for XON/XOFF flow control (if enabled)
    cmp     CfgData.FlowCtrl, 1 ; XON/XOFF mode?
    jne     SerialNotXONXOFF
    
    cmp     al, XOFF_CHAR       ; XOFF received?
    je      SerialXOFF
    cmp     al, XON_CHAR        ; XON received?
    je      SerialXON
    jmp     SerialNotXONXOFF
    
SerialXOFF:
    ; Pause transmission
    mov     TxPaused, 1
    jmp     SerialISR_Loop
    
SerialXON:
    ; Resume transmission
    mov     TxPaused, 0
    ; Check if we have pending data and restart TX
    cmp     TxPending, 0
    je      SerialISR_Loop
    ; Enable THRE interrupt to resume sending
    mov     dx, CfgData.Port
    add     dx, UART_IER
    in      al, dx
    or      al, 02h             ; Enable THRE interrupt (bit 1)
    out     dx, al
    jmp     SerialISR_Loop
    
SerialNotXONXOFF:
    ; Check for ACK
    cmp     al, ACK_CHAR
    je      HandleACK
    
    ; Check for NAK (retransmit request)
    cmp     al, NAK_CHAR
    je      HandleNAK
    
    ; Process received byte through frame parser
    call    ProcessReceivedByte
    jmp     SerialISR_Loop

HandleNAK:
    ; Read expected sequence number with timeout
    mov     dx, CfgData.Port
    push    ax
    mov     cx, 0FFFFh
WaitForNAKSeq:
    add     dx, UART_LSR
    in      al, dx
    test    al, 01h             ; Data ready?
    jnz     NAKSeqReady
    sub     dx, UART_LSR
    dec     cx
    jnz     WaitForNAKSeq
    pop     ax
    jmp     SerialISR_Loop      ; Timeout - ignore NAK
    
NAKSeqReady:
    sub     dx, UART_LSR
    in      al, dx              ; AL = expected sequence number
    pop     ax                  ; Restore
    
    ; Update NAK received statistics
    inc     word ptr MLIDStatistics.MRxOKMulticastCount
    adc     word ptr MLIDStatistics.MRxOKMulticastCount+2, 0
    
    ; Signal retransmit needed
    mov     TxRetransmit, 1
    ; Enable THRE interrupt to trigger retransmit
    mov     dx, CfgData.Port
    add     dx, UART_IER
    in      al, dx
    or      al, 02h
    out     dx, al
    jmp     SerialISR_Loop

HandleACK:
    ; Read sequence number with timeout
    mov     dx, CfgData.Port
    push    ax
    mov     cx, 0FFFFh          ; Timeout counter
WaitForSeq:
    add     dx, UART_LSR
    in      al, dx
    test    al, 01h             ; Data ready?
    jnz     SeqDataReady
    sub     dx, UART_LSR        ; Reset DX for next iteration
    dec     cx
    jnz     WaitForSeq
    jmp     ACKMismatch         ; Timeout - treat as mismatch
SeqDataReady:
    sub     dx, UART_LSR
    in      al, dx              ; Read sequence number
    
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
    
    ; Check if we should retry
    cmp     TxRetryCount, MAX_RETRIES
    jae     ACKRetriesExhausted
    
    ; Increment retry counter
    inc     TxRetryCount
    
    ; Update retry statistics
    inc     word ptr MLIDStatistics.MTxRetryCount
    adc     word ptr MLIDStatistics.MTxRetryCount+2, 0
    
    ; Retransmit the current frame
    push    es
    push    si
    les     si, dword ptr TxCurrentECB
    cmp     si, 0
    je      ACKNoRetry
    cmp     word ptr TxCurrentECB+2, 0
    je      ACKNoRetry
    
    ; Retransmit the frame (already built in TxFrameBuffer)
    call    TransmitFrame
    pop     si
    pop     es
    jmp     SerialISR_Loop
    
ACKNoRetry:
    pop     si
    pop     es
    jmp     SerialISR_Loop

ACKRetriesExhausted:
    ; Maximum retries reached - fail the ECB
    push    es
    push    si
    les     si, dword ptr TxCurrentECB
    cmp     si, 0
    je      ACKFailDone
    cmp     word ptr TxCurrentECB+2, 0
    je      ACKFailDone
    
    ; Mark ECB as failed
    mov     byte ptr es:[si+ECB_CompCode], ECB_TIMEOUT
    call    CompleteTransmitECB
    
    ; Update error statistics
    inc     word ptr MLIDStatistics.MTxMiscError
    adc     word ptr MLIDStatistics.MTxMiscError+2, 0
    
    ; Try next packet in queue
    call    StartNextTransmit
    
ACKFailDone:
    pop     si
    pop     es
    jmp     SerialISR_Loop

SerialISR_Done:
    ; Send EOI to PIC
    mov     al, 20h
    cmp     CfgData.IRQNum, 8
    jb      SerialEOI1
    
    out     0A0h, al            ; EOI to PIC2
SerialEOI1:
    out     020h, al            ; EOI to PIC1

    pop     ds
    pop     dx
    pop     cx
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
    push    cx
    push    dx
    push    ds

    mov     ax, cs              ; .COM file: CS=DS=ES
    mov     ds, ax

    ; Read status register to check interrupt source
    mov     dx, CfgData.Port
    add     dx, LPT_STATUS
    in      al, dx
    
    ; Check if nACK is low (data available in EPP mode)
    test    al, LPT_STAT_NACK
    jnz     ParISR_CheckTx      ; nACK high = no incoming data
    
    ; Read data from EPP data port
    mov     dx, CfgData.Port
    add     dx, LPT_EPP_DATA
    
    ; Read loop with timeout
    mov     cx, 0FFFFh          ; Timeout counter
ParISR_ReadLoop:
    ; Check if data ready (nBusy low = data valid)
    push    dx
    mov     dx, CfgData.Port
    add     dx, LPT_STATUS
    in      al, dx
    pop     dx
    test    al, LPT_STAT_NBUSY
    jz      ParISR_DataReady    ; nBusy low = data valid
    dec     cx
    jnz     ParISR_ReadLoop
    jmp     ParISR_Done         ; Timeout - no data
    
ParISR_DataReady:
    ; Read the byte from EPP data port
    in      al, dx
    
    ; Process through frame parser
    call    ProcessReceivedByte
    
    ; Check for more data (loop while nACK stays low)
    mov     dx, CfgData.Port
    add     dx, LPT_STATUS
    in      al, dx
    test    al, LPT_STAT_NACK
    jz      ParISR_ReadLoop     ; More data pending
    
ParISR_CheckTx:
    ; Check if remote signaled "not ready" (their INIT reflects as SELECT status)
    mov     dx, CfgData.Port
    add     dx, LPT_STATUS
    in      al, dx
    test    al, LPT_STAT_ONLINE         ; SELECT/Online bit = remote INIT state
    jz      ParISR_Done                 ; Remote not ready - don't send
    
    ; Check if transmit ready (nBusy high = can send)
    test    al, LPT_STAT_NBUSY
    jz      ParISR_Done         ; Busy - can't send now
    
    ; Check if we have pending TX data
    cmp     TxPending, 0
    je      ParISR_Done         ; No pending transmit
    
    ; Send next byte from TX buffer
    call    ParallelSendNextByte
    
ParISR_Done:
    ; Send EOI to PIC (handle PIC2 for IRQ >= 8)
    mov     al, 20h
    cmp     CfgData.IRQNum, 8
    jb      ParISR_EOI1
    out     0A0h, al            ; EOI to PIC2
ParISR_EOI1:
    out     020h, al            ; EOI to PIC1

    pop     ds
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    
    jmp     dword ptr cs:[OldHwInt_Off]
ParallelISR ENDP

;=============================================================================
; ParallelSendNextByte - Send next byte from TX buffer via parallel port
;=============================================================================
ParallelSendNextByte PROC
    push    ax
    push    dx
    push    si
    
    ; Get next byte from TX buffer
    mov     si, TxOutPtr
    cmp     si, TxInPtr
    je      ParSend_Empty       ; Buffer empty
    
    mov     al, TxFrameBuffer[si]
    inc     si
    cmp     si, TX_BUFFER_SIZE
    jb      ParSend_NoWrap
    xor     si, si              ; Wrap around
ParSend_NoWrap:
    mov     TxOutPtr, si
    
    ; Send byte via EPP data port
    mov     dx, CfgData.Port
    add     dx, LPT_EPP_DATA
    out     dx, al
    
    ; Check if buffer now empty
    mov     si, TxOutPtr
    cmp     si, TxInPtr
    jne     ParSend_Done
    mov     TxPending, 0        ; Clear pending flag
    
    ; Complete current ECB if any
    push    es
    les     si, dword ptr TxCurrentECB
    cmp     si, 0
    je      ParSend_NoECB
    cmp     word ptr TxCurrentECB+2, 0
    je      ParSend_NoECB
    mov     byte ptr es:[si+ECB_CompCode], ECB_SUCCESS
    call    CompleteTransmitECB
ParSend_NoECB:
    pop     es
    jmp     ParSend_Done
    
ParSend_Empty:
    mov     TxPending, 0
    
ParSend_Done:
    pop     si
    pop     dx
    pop     ax
    ret
ParallelSendNextByte ENDP

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
    ; Store and validate sequence number (should be 0 or 1)
    cmp     al, 1               ; Valid seq is 0 or 1
    ja      FrameError          ; Invalid sequence - reject early
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
    ; Validate length: must be > 0 and <= MTU_SIZE
    test    ax, ax
    jz      FrameError          ; Zero length - reject early
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
    
    ; Update ODI checksum error counter
    inc     word ptr MLIDStatistics.MRxChecksumError
    adc     word ptr MLIDStatistics.MRxChecksumError+2, 0
    
    ; Send NAK to request retransmit (serial mode only)
    cmp     CfgData.Mode, 0
    jne     ProcessDone             ; Skip NAK for parallel mode
    call    SendNAK

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
    ; Entries 0-31
    DW 00000h, 01021h, 02042h, 03063h, 04084h, 050A5h, 060C6h, 070E7h
    DW 08108h, 09129h, 0A14Ah, 0B16Bh, 0C18Ch, 0D1ADh, 0E1CEh, 0F1EFh
    DW 01231h, 00210h, 03273h, 02252h, 052B5h, 04294h, 072F7h, 062D6h
    DW 09339h, 08318h, 0B37Bh, 0A35Ah, 0D3BDh, 0C39Ch, 0F3FFh, 0E3DEh
    ; Entries 32-63
    DW 02462h, 03443h, 00420h, 01401h, 064E6h, 074C7h, 044A4h, 05485h
    DW 0A56Ah, 0B54Bh, 08528h, 09509h, 0E5EEh, 0F5CFh, 0C5ACh, 0D58Dh
    DW 03653h, 02672h, 01611h, 00630h, 076D7h, 066F6h, 05695h, 046B4h
    DW 0B75Bh, 0A77Ah, 09719h, 08738h, 0F7DFh, 0E7FEh, 0D79Dh, 0C7BCh
    ; Entries 64-95
    DW 048C4h, 058E5h, 06886h, 078A7h, 00840h, 01861h, 02802h, 03823h
    DW 0C9CCh, 0D9EDh, 0E98Eh, 0F9AFh, 08948h, 09969h, 0A90Ah, 0B92Bh
    DW 05AF5h, 04AD4h, 07AB7h, 06A96h, 01A71h, 00A50h, 03A33h, 02A12h
    DW 0DBFDh, 0CBDCh, 0FBBFh, 0EB9Eh, 09B79h, 08B58h, 0BB3Bh, 0AB1Ah
    ; Entries 96-127
    DW 06CA6h, 07C87h, 04CE4h, 05CC5h, 02C22h, 03C03h, 00C60h, 01C41h
    DW 0EDFEh, 0FDDFh, 0CDBCh, 0DD9Dh, 0AD7Ah, 0BD5Bh, 08D38h, 09D19h
    DW 07EC7h, 06EE6h, 05E85h, 04EA4h, 03E43h, 02E62h, 01E01h, 00E20h
    DW 0FFCFh, 0EFEEh, 0DF8Dh, 0CFACh, 0BF4Bh, 0AF6Ah, 09F09h, 08F28h
    ; Entries 128-159
    DW 09188h, 081A9h, 0B1CAh, 0A1EBh, 0D10Ch, 0C12Dh, 0F14Eh, 0E16Fh
    DW 01080h, 000A1h, 030C2h, 020E3h, 05004h, 04025h, 07046h, 06067h
    DW 083B9h, 09398h, 0A3FBh, 0B3DAh, 0C33Dh, 0D31Ch, 0E37Fh, 0F35Eh
    DW 002B1h, 01290h, 022F3h, 032D2h, 04235h, 05214h, 06277h, 07256h
    ; Entries 160-191
    DW 0B5EAh, 0A5CBh, 095A8h, 08589h, 0F56Eh, 0E54Fh, 0D52Ch, 0C50Dh
    DW 034E2h, 024C3h, 014A0h, 00481h, 07466h, 06447h, 05424h, 04405h
    DW 0A7DBh, 0B7FAh, 08799h, 097B8h, 0E75Fh, 0F77Eh, 0C71Dh, 0D73Ch
    DW 026D3h, 036F2h, 00691h, 016B0h, 06657h, 07676h, 04615h, 05634h
    ; Entries 192-223
    DW 0D94Ch, 0C96Dh, 0F90Eh, 0E92Fh, 099C8h, 089E9h, 0B98Ah, 0A9ABh
    DW 05844h, 04865h, 07806h, 06827h, 018C0h, 008E1h, 03882h, 028A3h
    DW 0CB7Dh, 0DB5Ch, 0EB3Fh, 0FB1Eh, 08BF9h, 09BD8h, 0ABBBh, 0BB9Ah
    DW 04A75h, 05A54h, 06A37h, 07A16h, 00AF1h, 01AD0h, 02AB3h, 03A92h
    ; Entries 224-255
    DW 0FD2Eh, 0ED0Fh, 0DD6Ch, 0CD4Dh, 0BDAAh, 0AD8Bh, 09DE8h, 08DC9h
    DW 07C26h, 06C07h, 05C64h, 04C45h, 03CA2h, 02C83h, 01CE0h, 00CC1h
    DW 0EF1Fh, 0FF3Eh, 0CF5Dh, 0DF7Ch, 0AF9Bh, 0BFBAh, 08FD9h, 09FF8h
    DW 06E17h, 07E36h, 04E55h, 05E74h, 02E93h, 03EB2h, 00ED1h, 01EF0h

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

    ; Update ODI statistics (required by GetMLIDStatistics)
    inc     word ptr MLIDStatistics.MTotalRxPackets
    adc     word ptr MLIDStatistics.MTotalRxPackets+2, 0

    ; Update legacy statistics
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
; DeliverFrameToLSL - Deliver received frame to LSL using ODI protocol
; Uses GetStackECB, HoldReceiveEvent, and ServiceEvents
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

    ; Set up segments (.COM file)
    mov     ax, cs
    mov     ds, ax

    ; Received frame is in FrameBuffer+3 (skip seq + 2-byte length)
    ; Frame format: [dest MAC 6][src MAC 6][type/len 2][data...]
    ; RxFrameLen = total data length after our header

    ; Fill in LookAheadStruc for GetStackECB
    ; LMediaHeaderPtr = pointer to Ethernet header (first 14 bytes)
    lea     ax, FrameBuffer+3
    mov     word ptr RxLookAhead.LMediaHeaderPtr[0], ax
    mov     word ptr RxLookAhead.LMediaHeaderPtr[2], cs

    ; LookAheadPtr = pointer to packet data after media header
    ; For Ethernet, data starts at offset 14 (after MAC headers + type)
    lea     ax, FrameBuffer+3+14
    mov     word ptr RxLookAhead.LookAheadPtr[0], ax
    mov     word ptr RxLookAhead.LookAheadPtr[2], cs

    ; LookAheadLen = min(CurrentLookAhead, data_length)
    mov     ax, RxFrameLen
    sub     ax, 14              ; Subtract media header size
    jbe     RxTooShort          ; Frame too short
    cmp     ax, CurrentLookAhead
    jbe     LookAheadOK2
    mov     ax, CurrentLookAhead
LookAheadOK2:
    mov     RxLookAhead.LookAheadLen, ax

    ; LProtID = Protocol ID (for Ethernet_802.2, this is DSAP/SSAP/Control)
    ; Copy first 6 bytes of data as protocol ID (3 words)
    push    si
    push    di
    lea     si, FrameBuffer+3+14    ; Point to data after media header
    lea     di, RxLookAhead.LProtID
    mov     cx, 3
    rep     movsw
    pop     di
    pop     si

    ; LBoardNum = our registered board number
    mov     ax, LSLBoardNum
    mov     RxLookAhead.LBoardNum, ax

    ; LDataSize = total packet size (including media header)
    mov     ax, RxFrameLen
    mov     RxLookAhead.LDataSize, ax

    ; LImmAddress = source MAC address (6 bytes = 3 words at offset 6)
    push    si
    push    di
    lea     si, FrameBuffer+3+6     ; Source MAC at offset 6
    lea     di, RxLookAhead.LImmAddress
    mov     cx, 3
    rep     movsw
    pop     di
    pop     si

    ; LPacketAttrib = 0 (normal packet)
    mov     RxLookAhead.LPacketAttrib, 0

    ; LDestType = determine from destination MAC
    ; 0 = unicast, 1 = multicast, 2 = broadcast
    mov     al, byte ptr FrameBuffer+3  ; First byte of dest MAC
    test    al, 01h                     ; Multicast bit?
    jz      DestUnicast
    ; Check for broadcast (FF:FF:FF:FF:FF:FF)
    cmp     word ptr FrameBuffer+3, 0FFFFh
    jne     DestMulticast
    cmp     word ptr FrameBuffer+5, 0FFFFh
    jne     DestMulticast
    cmp     word ptr FrameBuffer+7, 0FFFFh
    jne     DestMulticast
    mov     RxLookAhead.LDestType, 2    ; Broadcast
    jmp     DestTypeDone
DestMulticast:
    mov     RxLookAhead.LDestType, 1    ; Multicast
    jmp     DestTypeDone
DestUnicast:
    mov     RxLookAhead.LDestType, 0    ; Unicast
DestTypeDone:

    ; LStartCopyOffset = 0 (protocol will set this)
    mov     RxLookAhead.LStartCopyOffset, 0

    ; LPriorityLevel = 0
    mov     RxLookAhead.LPriorityLevel, 0

    ; Call GetStackECB (BX=10h) to get ECB from protocol stack
    ; DS:DI = pointer to LookAheadStruc
    ; Returns: AX = status, ES:SI = ECB pointer
    cli                             ; Disable interrupts per spec
    lea     di, RxLookAhead
    mov     bx, MLIDSUP_GET_STACK_ECB   ; 10h
    call    dword ptr [LSLInfoBuf.LSLEntryPoint]
    
    test    ax, ax
    jnz     NoECBAvailable          ; No ECB available

    ; ES:SI now points to the ECB from protocol stack
    ; Copy packet data into ECB fragment buffers
    ; ECB fragment list starts at ECB+36 (ECB_FragList)
    ; Fragment structure: DD address (4), DW size (2) = 6 bytes per fragment
    
    ; Save ECB pointer
    mov     word ptr cs:[SavedECBOff], si
    mov     word ptr cs:[SavedECBSeg], es

    ; Check fragment count
    mov     cx, es:[si+ECB_FragCount]
    test    cx, cx
    jz      CopyDone                ; No fragments

    ; Get first fragment info (ES:SI + ECB_FragList)
    add     si, ECB_FragList
    
    ; First fragment: [SI] = DD address, [SI+4] = DW size
    mov     ax, es:[si+4]           ; AX = fragment buffer size
    mov     cx, RxFrameLen          ; CX = packet data size
    cmp     ax, cx
    jbe     UseFragSize
    mov     ax, cx                  ; Use smaller of the two
UseFragSize:
    mov     cx, ax                  ; CX = bytes to copy

    ; Load destination address from fragment
    les     di, dword ptr es:[si]   ; ES:DI = fragment buffer
    
    ; Set up source (CS:FrameBuffer+3)
    push    cs
    pop     ds
    lea     si, FrameBuffer+3       ; DS:SI = source packet data
    
    ; Copy data (use word moves for performance)
    shr     cx, 1               ; Divide by 2 for word count
    rep     movsw               ; Copy words
    adc     cx, cx              ; CX=1 if odd byte remains
    rep     movsb               ; Copy trailing byte if any

CopyDone:
    ; Restore ECB pointer
    mov     si, word ptr cs:[SavedECBOff]
    mov     es, word ptr cs:[SavedECBSeg]
    
    ; Restore DS
    push    cs
    pop     ds
    
    ; Set completion code to success
    mov     byte ptr es:[si+ECB_CompCode], ECB_SUCCESS
    
    ; Clear InUse flag
    mov     byte ptr es:[si+ECB_InUse], 0

    ; Call HoldReceiveEvent (BX=7) to queue completed ECB
    ; ES:SI = pointer to ECB
    mov     bx, MLIDSUP_HOLD_RECV_EVENT  ; 7
    call    dword ptr [LSLInfoBuf.LSLEntryPoint]

    ; Call ServiceEvents (BX=0Bh) to process queued events
    mov     bx, MLIDSUP_SERVICE_EVENTS   ; 0Bh
    call    dword ptr [LSLInfoBuf.LSLEntryPoint]

    ; Resume remote if we had paused (buffer space now available)
    cmp     RxPaused, 0
    je      DeliverDone
    call    ResumeBackpressure

    jmp     DeliverDone

NoECBAvailable:
    ; Update statistics - no ECB available
    inc     word ptr MLIDStatistics.MNoECBsAvailable
    adc     word ptr MLIDStatistics.MNoECBsAvailable+2, 0
    
    ; Signal sender to pause (backpressure)
    call    SendBackpressure
    jmp     DeliverDone

RxTooShort:
    ; Frame too short
    inc     word ptr MLIDStatistics.MRxTooSmall
    adc     word ptr MLIDStatistics.MRxTooSmall+2, 0

DeliverDone:
    sti                             ; Re-enable interrupts

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
    
    ; Wait for transmit ready with timeout
    mov     cx, 0FFFFh          ; Timeout counter
WaitTxReady:
    push    cx                  ; Preserve counter
    add     dx, UART_LSR
    in      al, dx
    test    al, 20h             ; Transmit ready?
    pop     cx
    jnz     TxReady1
    sub     dx, UART_LSR        ; Reset DX for next iteration
    dec     cx
    jnz     WaitTxReady
    jmp     SendACKDone         ; Timeout - abort
TxReady1:
    ; Send ACK
    sub     dx, UART_LSR
    mov     al, ACK_CHAR
    out     dx, al

    ; Send sequence number with timeout
    mov     cx, 0FFFFh          ; Timeout counter
WaitTxReady2:
    push    cx                  ; Preserve counter
    add     dx, UART_LSR
    in      al, dx
    test    al, 20h             ; Transmit ready?
    pop     cx
    jnz     TxReady2
    sub     dx, UART_LSR        ; Reset DX for next iteration
    dec     cx
    jnz     WaitTxReady2
    jmp     SendACKDone         ; Timeout - abort
TxReady2:
    sub     dx, UART_LSR
    mov     al, ah
    out     dx, al

SendACKDone:
    pop     dx
    pop     ax
    ret
SendACK ENDP

;=============================================================================
; SendNAK - Send NAK character requesting retransmission (serial only)
; Uses NAK_CHAR (0x15) followed by expected sequence number
;=============================================================================
SendNAK PROC
    push    ax
    push    dx
    push    cx
    
    mov     dx, CfgData.Port
    
    ; Wait for transmit ready
    mov     cx, 0FFFFh
WaitNAKReady1:
    push    cx
    add     dx, UART_LSR
    in      al, dx
    test    al, 20h
    pop     cx
    jnz     NAKTxReady1
    sub     dx, UART_LSR
    dec     cx
    jnz     WaitNAKReady1
    jmp     SendNAKDone
    
NAKTxReady1:
    sub     dx, UART_LSR
    mov     al, NAK_CHAR
    out     dx, al
    
    ; Send expected sequence number
    mov     cx, 0FFFFh
WaitNAKReady2:
    push    cx
    add     dx, UART_LSR
    in      al, dx
    test    al, 20h
    pop     cx
    jnz     NAKTxReady2
    sub     dx, UART_LSR
    dec     cx
    jnz     WaitNAKReady2
    jmp     SendNAKDone
    
NAKTxReady2:
    sub     dx, UART_LSR
    mov     al, RxSeqNum              ; Send expected seq num
    out     dx, al
    
    ; Update NAK statistics
    inc     word ptr MLIDStatistics.MTxOKMulticastCount
    adc     word ptr MLIDStatistics.MTxOKMulticastCount+2, 0

SendNAKDone:
    pop     cx
    pop     dx
    pop     ax
    ret
SendNAK ENDP

;=============================================================================
; Backpressure Flow Control Routines
; Signal remote to pause/resume when buffers fill/empty
;=============================================================================

;=============================================================================
; SendBackpressure - Signal remote to pause (mode-independent dispatcher)
;=============================================================================
SendBackpressure PROC
    push    ax
    
    cmp     RxPaused, 1                 ; Already paused?
    je      BackpressureDone
    
    cmp     CfgData.Mode, 0             ; Serial mode?
    je      BackpressureSerial
    ; Parallel mode (PAR4 or PAR8)
    call    ParSendPause
    jmp     BackpressureDone
    
BackpressureSerial:
    call    SerialSendPause
    
BackpressureDone:
    pop     ax
    ret
SendBackpressure ENDP

;=============================================================================
; ResumeBackpressure - Signal remote to resume (mode-independent dispatcher)
;=============================================================================
ResumeBackpressure PROC
    push    ax
    
    cmp     RxPaused, 0                 ; Not paused?
    je      ResumeDone
    
    cmp     CfgData.Mode, 0             ; Serial mode?
    je      ResumeSerial
    ; Parallel mode (PAR4 or PAR8)
    call    ParSendResume
    jmp     ResumeDone
    
ResumeSerial:
    call    SerialSendResume
    
ResumeDone:
    pop     ax
    ret
ResumeBackpressure ENDP

;=============================================================================
; SerialSendPause - Pause remote via XOFF or lowering RTS (serial mode)
;=============================================================================
SerialSendPause PROC
    push    ax
    push    dx
    
    cmp     CfgData.FlowCtrl, 1         ; XON/XOFF mode?
    jne     SerialPauseHardware
    
    ; Software flow: send XOFF character
    mov     dx, CfgData.Port
    add     dx, UART_LSR
WaitXOFFReady:
    in      al, dx
    test    al, 20h                     ; THRE ready?
    jz      WaitXOFFReady
    sub     dx, UART_LSR
    mov     al, XOFF_CHAR
    out     dx, al
    jmp     SerialPauseSet

SerialPauseHardware:
    ; Hardware flow: lower RTS
    mov     dx, CfgData.Port
    add     dx, UART_MCR
    in      al, dx
    and     al, 0FDh                    ; Clear RTS bit (NOT MCR_RTS)
    out     dx, al

SerialPauseSet:
    mov     RxPaused, 1
    
    pop     dx
    pop     ax
    ret
SerialSendPause ENDP

;=============================================================================
; SerialSendResume - Resume remote via XON or raising RTS (serial mode)
;=============================================================================
SerialSendResume PROC
    push    ax
    push    dx
    
    cmp     CfgData.FlowCtrl, 1         ; XON/XOFF mode?
    jne     SerialResumeHardware
    
    ; Software flow: send XON character
    mov     dx, CfgData.Port
    add     dx, UART_LSR
WaitXONReady:
    in      al, dx
    test    al, 20h                     ; THRE ready?
    jz      WaitXONReady
    sub     dx, UART_LSR
    mov     al, XON_CHAR
    out     dx, al
    jmp     SerialResumeClear

SerialResumeHardware:
    ; Hardware flow: raise RTS
    mov     dx, CfgData.Port
    add     dx, UART_MCR
    in      al, dx
    or      al, MCR_RTS                 ; Set RTS bit
    out     dx, al

SerialResumeClear:
    mov     RxPaused, 0
    
    pop     dx
    pop     ax
    ret
SerialSendResume ENDP

;=============================================================================
; ParSendPause - Signal remote to pause (parallel mode)
; Uses INIT line (active low) - lowering it signals "not ready"
;=============================================================================
ParSendPause PROC
    push    ax
    push    dx
    
    ; Lower INIT line to signal "not ready to receive"
    mov     dx, CfgData.Port
    add     dx, LPT_CONTROL
    in      al, dx
    and     al, 0FBh                    ; Clear INIT bit (NOT LPT_CTRL_INIT)
    out     dx, al
    
    mov     RxPaused, 1
    
    pop     dx
    pop     ax
    ret
ParSendPause ENDP

;=============================================================================
; ParSendResume - Signal remote to resume (parallel mode)
; Raises INIT line to signal "ready to receive"
;=============================================================================
ParSendResume PROC
    push    ax
    push    dx
    
    ; Raise INIT line to signal "ready to receive"
    mov     dx, CfgData.Port
    add     dx, LPT_CONTROL
    in      al, dx
    or      al, LPT_CTRL_INIT           ; Set INIT bit
    out     dx, al
    
    mov     RxPaused, 0
    
    pop     dx
    pop     ax
    ret
ParSendResume ENDP

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
    
    ; Reset retry counter for new transmission
    mov     TxRetryCount, 0

    ; Build packet frame from ECB fragments
    call    BuildTransmitFrame
    jc      TransmitError

    ; Start transmitting the frame
    call    TransmitFrame

    ; Complete ECB immediately after transmission
    ; (For now, don't wait for ACK - allows IPXODI to work without peer)
    mov     byte ptr es:[si+ECB_CompCode], ECB_SUCCESS
    call    CompleteTransmitECB

    ; Start next transmission if queued
    cmp     word ptr TxQueueHead, 0
    je      StartDone
    cmp     word ptr TxQueueHead+2, 0
    je      StartDone
    jmp     HasPacket           ; Process next queued ECB

TransmitError:
    ; Complete ECB with error
    les     si, dword ptr TxCurrentECB
    mov     byte ptr es:[si+ECB_CompCode], ECB_HARDWARE
    call    CompleteTransmitECB
    
    ; Try next packet
    jmp     HasPacket

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
    
    ; Copy fragment data (use word moves for performance)
    shr     cx, 1               ; Divide by 2 for word count
    rep     movsw               ; Copy words
    adc     cx, cx              ; CX=1 if odd byte remains
    rep     movsb               ; Copy trailing byte if any
    
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
    
    ; Update ODI statistics (required by GetMLIDStatistics)
    inc     word ptr MLIDStatistics.MTotalTxPackets
    adc     word ptr MLIDStatistics.MTotalTxPackets+2, 0
    
    ; Update legacy statistics
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
; CompleteTransmitECB - Complete ECB and remove from queue using ODI protocol
; Input: ES:SI = ECB, CompCode already set
; Uses SendComplete (BX=0Eh) and ServiceEvents (BX=0Bh)
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
    
    ; Clear current ECB tracking
    mov     word ptr TxCurrentECB, 0
    mov     word ptr TxCurrentECB+2, 0
    mov     TxPending, 0

    ; Use ODI SendComplete to queue ECB for ESR dispatch
    ; BX = 0Eh (MLIDSUP_SEND_COMPLETE)
    ; ES:SI = pointer to ECB
    cli                             ; Interrupts must be disabled
    mov     bx, MLIDSUP_SEND_COMPLETE   ; 0Eh
    call    dword ptr [LSLInfoBuf.LSLEntryPoint]
    
    ; Call ServiceEvents to dispatch queued ESRs
    mov     bx, MLIDSUP_SERVICE_EVENTS  ; 0Bh
    call    dword ptr [LSLInfoBuf.LSLEntryPoint]
    
    sti                             ; Re-enable interrupts
    
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
    ; BX = function code (ODI standard per spec page 18-10 to 18-20)
    ; Function 0: GetMLIDConfiguration (required)
    ; Function 1: GetMLIDStatistics (required)
    ; Function 4: RegisterTxMonitor (required)
    ; Function 5: MLIDShutdown (required)
    ; Function 6: MLIDReset (required)
    ; Function 9: SetLookAheadSize (required)
    ; Function 10: PromiscuousChange (recommended)
    
    cmp     bx, 0               ; GetMLIDConfiguration
    je      DoGetConfig
    cmp     bx, 1               ; GetMLIDStatistics
    je      DoGetStats
    cmp     bx, 4               ; RegisterTxMonitor
    je      DoRegisterTxMon
    cmp     bx, 5               ; MLIDShutdown
    je      DoShutdown
    cmp     bx, 6               ; MLIDReset
    je      DoReset
    cmp     bx, 9               ; SetLookAheadSize
    je      DoSetLookAhead
    cmp     bx, 10              ; PromiscuousChange
    je      DoPromiscuous
    
    ; Unsupported function
    mov     ax, 8008h           ; LSLERR_BAD_COMMAND
    retf

DoGetConfig:
    ; Return ES:SI pointing to MLIDConfigTable
    ; This is how IPXODI discovers and binds to boards!
    push    cs
    pop     es
    lea     si, MLIDConfigTable
    xor     ax, ax              ; AX=0 = success
    retf

DoGetStats:
    ; Return ES:SI pointing to MLIDStatistics table
    push    cs
    pop     es
    lea     si, MLIDStatistics
    xor     ax, ax              ; AX=0 = success
    retf

DoRegisterTxMon:
    ; CX = 0 to disable, non-zero to enable
    ; ES:SI = pointer to TX monitor routine
    test    cx, cx
    jz      DisableTxMon
    
    ; Check if already registered
    cmp     TxMonitorActive, 0
    jne     TxMonAlreadyReg
    
    ; Register TX monitor
    mov     TxMonitorOff, si
    mov     TxMonitorSeg, es
    mov     TxMonitorActive, 1
    xor     ax, ax              ; AX=0 = success
    retf
    
TxMonAlreadyReg:
    mov     ax, 8001h           ; LSLERR_OUT_OF_RESOURCES
    retf
    
DisableTxMon:
    ; Verify ES:SI matches registered monitor
    cmp     si, TxMonitorOff
    jne     TxMonBadParam
    mov     ax, es
    cmp     ax, TxMonitorSeg
    jne     TxMonBadParam
    
    ; Disable TX monitor
    mov     TxMonitorActive, 0
    mov     TxMonitorOff, 0
    mov     TxMonitorSeg, 0
    xor     ax, ax              ; AX=0 = success
    retf
    
TxMonBadParam:
    mov     ax, 8002h           ; LSLERR_BAD_PARAMETER
    retf

DoReset:
    ; Reset hardware and buffers
    call    InitSerialPort
    mov     RxState, STATE_IDLE
    mov     RxBufIdx, 0
    mov     TxPending, 0
    xor     ax, ax              ; AX=0 = success
    retf

DoShutdown:
    ; CX=0: permanent shutdown, CX non-zero: temporary
    ; Disable UART interrupts
    push    dx
    mov     dx, CfgData.Port
    add     dx, UART_IER
    xor     al, al
    out     dx, al
    pop     dx
    xor     ax, ax              ; AX=0 = success
    retf

DoSetLookAhead:
    ; CX = requested lookahead size (0-128)
    ; Only increase, never decrease
    cmp     cx, CurrentLookAhead
    jbe     LookAheadOK
    cmp     cx, 128             ; Max 128 per spec
    ja      LookAheadOK
    mov     CurrentLookAhead, cx
    ; Update config table
    mov     MLIDConfigTable.LookAheadSize, cx
LookAheadOK:
    xor     ax, ax              ; AX=0 = success
    retf

DoPromiscuous:
    ; CX = promiscuous mode (0=off, 1=on)
    ; For serial link, promiscuous mode not applicable
    xor     ax, ax              ; AX=0 = success
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
    
    ; Look up baud rate and print appropriate string
    ; Baud rate is 32-bit, compare against known values
    mov     ax, word ptr CfgData.BaudRate
    mov     dx, word ptr CfgData.BaudRate+2
    
    ; Check 9600 (0x00002580)
    cmp     dx, 0
    jne     CheckHighBaud
    cmp     ax, 9600
    jne     Check19200
    lea     dx, MsgBaud9600
    jmp     PrintBaudStr
    
Check19200:
    cmp     ax, 19200
    jne     Check38400
    lea     dx, MsgBaud19200
    jmp     PrintBaudStr
    
Check38400:
    cmp     ax, 38400
    jne     Check57600
    lea     dx, MsgBaud38400
    jmp     PrintBaudStr
    
Check57600:
    cmp     ax, 57600
    jne     CheckHighBaud
    lea     dx, MsgBaud57600
    jmp     PrintBaudStr
    
CheckHighBaud:
    ; Check 115200 (0x0001C200)
    cmp     dx, 1
    jne     BaudUnknown
    cmp     ax, 0C200h
    jne     BaudUnknown
    lea     dx, MsgBaud115200
    jmp     PrintBaudStr
    
BaudUnknown:
    lea     dx, MsgBaudUnknown
    
PrintBaudStr:
    mov     ah, 09h
    int     21h

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
    
    ; Check for deregistration error
    test    ax, ax
    jz      DeregisterOK
    
    ; Deregistration failed - print warning and continue
    push    ax
    lea     dx, MsgWarnDeregister
    mov     ah, 09h
    int     21h
    pop     ax
    ; Continue anyway - driver may be partially unloaded
    
DeregisterOK:
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
    
    ; Restore INT 1Ch (timer) if it was hooked for verbose mode
    mov     es, Unload_TSRSeg
    mov     ax, es:[OldInt1C_Off]
    test    ax, ax
    jz      SkipRestoreTimer        ; Not hooked if OldInt1C_Off == 0
    mov     dx, ax
    mov     ax, es:[OldInt1C_Seg]
    push    ds
    mov     ds, ax
    mov     ax, 251Ch
    int     21h
    pop     ds
SkipRestoreTimer:

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

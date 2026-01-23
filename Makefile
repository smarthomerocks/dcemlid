# Makefile for DCEMLID - ODI MLID Driver
# Supports both DOS (Borland TASM) and Linux (NASM + dosemu2)

# Default target
.PHONY: all clean dos linux install help

# Source files
SRC = DCEMLID.ASM
OBJ = DCEMLID.OBJ
EXE = DCEMLID.EXE
COM = DCEMLID.COM
MAP = DCEMLID.MAP

# Borland TASM/TLINK paths (adjust as needed)
TASM = TASM.EXE
TLINK = TLINK.EXE
EXE2BIN = EXE2BIN.EXE

# NASM and tools for Linux
NASM = nasm
OBJCOPY = objcopy

# Assembler flags
TASM_FLAGS = /m2 /zi /l
TLINK_FLAGS = /t /x
NASM_FLAGS = -f bin -o $(COM)

# Help target (default)
help:
	@echo "DCEMLID Makefile"
	@echo "================"
	@echo ""
	@echo "Targets:"
	@echo "  make dos     - Build using Borland TASM (DOS/Windows)"
	@echo "  make linux   - Build using NASM (Linux/Unix)"
	@echo "  make clean   - Remove generated files"
	@echo "  make help    - Display this help"
	@echo ""
	@echo "Requirements:"
	@echo "  DOS:   Borland TASM 3.0+, TLINK, EXE2BIN"
	@echo "  Linux: NASM 2.0+, GNU objcopy"
	@echo ""
	@echo "For detailed build instructions, see BUILD.TXT"

# Build with Borland TASM (DOS/Windows)
dos: $(COM)
	@echo "Build complete: $(COM)"
	@echo "Copy to DOS system and run: DCEMLID /?"

$(OBJ): $(SRC)
	$(TASM) $(TASM_FLAGS) $(SRC)

$(EXE): $(OBJ)
	$(TLINK) $(TLINK_FLAGS) $(OBJ)

$(COM): $(EXE)
	$(EXE2BIN) $(EXE) $(COM)

# Build with NASM (Linux)
linux: $(SRC)
	@echo "Note: NASM build requires modified source for 16-bit DOS binary output"
	@echo "Converting assembly syntax..."
	@echo "Building with NASM..."
	$(NASM) $(NASM_FLAGS) $(SRC) || echo "Error: NASM build failed. Use DOS build or convert syntax."

# Alternative: Build using DOSBox/dosemu2 with Borland tools on Linux
dosbox: $(SRC)
	@echo "Building with Borland TASM via DOSBox..."
	@if [ -f /usr/bin/dosbox ]; then \
		dosbox -c "mount c ." -c "c:" -c "tasm $(TASM_FLAGS) $(SRC)" -c "tlink $(TLINK_FLAGS) $(OBJ)" -c "exe2bin $(EXE) $(COM)" -c "exit"; \
	else \
		echo "Error: DOSBox not found. Install with: sudo apt-get install dosbox"; \
		exit 1; \
	fi

# Clean build artifacts
clean:
	@echo "Cleaning build files..."
	-rm -f $(OBJ) $(EXE) $(COM) $(MAP) DCEMLID.LST
	-del $(OBJ) $(EXE) $(COM) $(MAP) DCEMLID.LST 2>NUL

# Install target (DOS only - copies to current directory)
install: $(COM)
	@echo "Installation:"
	@echo "  1. Copy $(COM) to your DOS networking directory"
	@echo "  2. Ensure LSL.COM is loaded first"
	@echo "  3. Run: $(COM) /?"
	@echo ""
	@echo "Example AUTOEXEC.BAT:"
	@echo "  CD \\NWCLIENT"
	@echo "  LSL"
	@echo "  DCEMLID /MODE=SERIAL /PORT=3F8 /BAUD=115200"
	@echo "  IPXODI"

# Development target - build and test
test: dos
	@echo "Testing requires DOS environment"
	@echo "Run in DOSBox or real DOS system"

# Distribution package
dist: dos
	@echo "Creating distribution package..."
	-mkdir -p dist
	cp $(COM) dist/
	cp DCEMLID.TXT dist/
	cp BUILD.TXT dist/
	cp README.md dist/ 2>/dev/null || echo "README.md not found, skipping"
	@echo "Distribution created in dist/"

# Dependencies
$(COM): $(SRC)

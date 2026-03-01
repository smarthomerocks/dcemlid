# Makefile for DCEMLID - ODI MLID Driver
# For Borland TASM 5.0 Make

# Source files
SRC = DCEMLID.ASM
OBJ = DCEMLID.OBJ
COM = DCEMLID.COM

# Borland TASM/TLINK paths
TASM = TASM.EXE
TLINK = TLINK.EXE

# Assembler flags
# /m2 = multiple passes for forward references
# /zi = include debug info
# /l = generate listing file
# /jJUMPS = auto-convert short jumps to near when needed
TASM_FLAGS = /m2 /zi /l /jJUMPS
TLINK_FLAGS = /t /x

# Default target - build the COM file
all: $(COM)

# Build COM file directly with TLINK /t (no EXE2BIN needed)
$(COM): $(OBJ)
	$(TLINK) $(TLINK_FLAGS) $(OBJ)
	@echo Build complete: $(COM)

$(OBJ): $(SRC)
	$(TASM) $(TASM_FLAGS) $(SRC)

# Clean build artifacts  
clean:
   -del $(OBJ)
   -del $(COM)
   -del DCEMLID.MAP
   -del DCEMLID.LST

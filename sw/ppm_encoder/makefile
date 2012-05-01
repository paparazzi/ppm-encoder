#Compatible with all versions of AVRGCC makefile (i hope!)
#Written by Chris Efstathiou 
#Date 29/Oct/2009


###### define some variables based on the AVR base path in $(AVR) #######
ifneq "$(wine)"""
	WINE	= wine
	CC	= wine avr-gcc
	AS	= wine avr-gcc -x assembler-with-cpp	
	RM	= rm -f
	RN	= mv
	BIN	= wine avr-objcopy
	SIZE	= wine avr-size
	MERGE	= wine srec_cat
else 
	CC	= avr-gcc
	AS	= avr-gcc -x assembler-with-cpp	
	RM	= rm -f
	RN	= mv
	BIN	= avr-objcopy
	SIZE	= avr-size
	MERGE	= srec_cat
endif

#Use the below commented line instead of the above line if you use avr-sizex
	AVRDUDE = avrdude
	INCDIR	= .
	LIBDIR	= $(AVR)/avr/lib
	NORMAL_PROG_MEM_START = 0


######################################################################################
#           U S E R   C O N F I G U R A T I O N  B L O C K  S T A R T                  
######################################################################################

#put the MCU clock Frequency in Hertz here (This setting is passed to all files)
	F_CPU = 16000000UL


#put the name of the target mcu here (at90s8515, at90s8535, attiny22, atmega603 etc.)
ifneq "$(mcu)"""
ifeq "$(mcu)""atmega168"
	MCU = atmega168
#put your appropriate Bootloader file name here (hex type with the .hex extension) 
        BOOTLOADER = bootloader-m168-8mhz.hex
	# The below fuse settings are for the mega168
	#Low fuse has the same functions for the mega 328 and mega 168
        #LOW_FUSE_168 = 0xC2
	#High fuse sets sets the bootsize on 328 and the bodlevel in 168
	HIGH_FUSE_168 = 0xDC
	#Extended fuse sets the bodlevel only on 328 but it sets the bootsize on 168
	EXTENDED_FUSE_168 = 0xF8
	LOCK_FUSE_168 = 0xFF
endif
else 
	MCU = atmega328p
#put your appropriate Bootloader file name here (hex type with the .hex extension) 
        BOOTLOADER = bootloader-m328p-16mhz.hex
	# The below fuse settings are for the 328p
	#Low fuse has the same functions for the mega 328 and mega 168
        #LOW_FUSE_328 = 0xDF
	#High fuse sets sets the bootsize on 328 and the bodlevel in 168
	HIGH_FUSE_328 = 0xD8
	#Extended fuse sets the bodlevel only on 328 but it sets the bootsize on 168
	EXTENDED_FUSE_328 = 0xFC
	LOCK_FUSE_328 = 0xFF
endif

ifeq "$(F_CPU)""16000000UL"
        LOW_FUSE_328 = 0xDF
	LOW_FUSE_168 = 0xDF
endif
ifeq "$(F_CPU)""8000000UL"
        LOW_FUSE_328 = 0xC2
	LOW_FUSE_168 = 0xC2
endif

# Edit the below application start address if the program is a bootloader.
# 0 for starting at address 0, 0x3800 is the BYTE!!! address not the word address (0x1C00)
# of the mega168  bootloader start address (databook bootloader address * 2 = byte address).
# Set this to 0 for normal applications
        MEM_START = 0

#put the name of the target file here (without extension)
        TRG     = ppm_encoder_v4_3

#put your C sourcefiles here 
        SRC     = $(TRG).c  
        
#put additional assembler source file here
        ASRC    =   

#additional libraries and object files to link
        LIB     =

#additional includes to compile
        INC     = 

###### output format can be srec, ihex (avrobj is always created) #######
	FORMAT	= ihex

# Programming with avrdude settings, be carefull !!!
# Uncomment the lines you need.

AVRDUDE_PROGRAMMER = stk500v2
AVRDUDE_PORT = /dev/ttyUSB0

ifneq "$(file)"""
AVRDUDE_WRITE_FLASH = -U flash:w:$(file)
else 
AVRDUDE_WRITE_FLASH = -U flash:w:$(TRG).hex
endif


#AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TRG).eep

AVRDUDE_OVERIDE_MCU_TYPE = -F

# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
#AVRDUDE_NO_VERIFY = -V

# Increase verbosity level.
#AVRDUDE_VERBOSE = -v -v

AVRDUDE_BASIC = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FLAGS = $(AVRDUDE_BASIC) $(AVRDUDE_OVERIDE_MCU_TYPE) $(AVRDUDE_NO_VERIFY) $(AVRDUDE_VERBOSE) $(AVRDUDE_ERASE_COUNTER)

#---------------- Programming Options (avrdude) ----------------

# Uncomment the following if you use serial port and you need
# an other baudrate than the default for the programmer
#AVRDUDE_SET_BAUDRATE = -b
#AVRDUDE_BAUDRATE = 38400

AVRDUDE_WRITE_FLASH = -U flash:w:$(TRG).hex
#AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep
AVRDUDE_READ_FUSE = -U lfuse:r:-:i -U hfuse:r:-:i -U efuse:r:-:i -U lock:r:-:i

#AVRDUDE_WRITE_FUSE = -U lfuse:w:$(LOW_FUSE):m -U hfuse:w:$(HIGH_FUSE):m -U efuse:w:$(EXTENDED_FUSE):m -U lock:w:$(LOCK_FUSE):m
ifneq "$(mcu)"""
	AVRDUDE_WRITE_FUSE = -U lfuse:w:$(LOW_FUSE_168):m -U hfuse:w:$(HIGH_FUSE_168):m -U efuse:w:$(EXTENDED_FUSE_168):m
else 
	AVRDUDE_WRITE_FUSE = -U lfuse:w:$(LOW_FUSE_328):m -U hfuse:w:$(HIGH_FUSE_328):m -U efuse:w:$(EXTENDED_FUSE_328):m  
endif


# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
#AVRDUDE_NO_VERIFY = -V

# Increase verbosity level. Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude>
# to submit bug reports.
#AVRDUDE_VERBOSE = -v -v

AVRDUDE_DISABLE_SAFEMODE = -u
AVRDUDE_QUELL_PROGRESS_OUTPUT = -q

AVRDUDE_FLAGS = -p $(MCU)
AVRDUDE_FLAGS += -P $(AVRDUDE_PORT) 
AVRDUDE_FLAGS += -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)

AVRDUDE_FUSE_FLAGS = -p $(MCU) $(AVRDUDE_DISABLE_SAFEMODE)
AVRDUDE_FUSE_FLAGS += -P $(AVRDUDE_PORT) 
AVRDUDE_FUSE_FLAGS += -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FUSE_FLAGS += $(AVRDUDE_QUELL_PROGRESS_OUTPUT) 



######################################################################################
#           U S E R   C O N F I G U R A T I O N  B L O C K  E N D                  
######################################################################################

########## Normally you shouldn't need to change anything in the below lines #########

#compiler flags
      CPFLAGS = -g -Os -fverbose-asm --param inline-call-cost=2 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-ahlms=$(<:.c=.lst)

#Assembler flags
	ASFLAGS = -Wa,-gstabs  

#linker flags
        LDFLAGS = -Wl,-Map=$(TRG).map,--cref 
        

#define all project specific object files
	OBJ	= $(ASRC:.s=.o) $(SRC:.c=.o) 
	CPFLAGS += -mmcu=$(MCU) 
	ASFLAGS += -mmcu=$(MCU) 
	LDFLAGS += -mmcu=$(MCU) 
	ifneq "$(MEM_START)""$(NORMAL_PROG_MEM_START)"
	LDFLAGS += -Wl,--section-start=.text=$(MEM_START)
	endif

#Pass the MPU clock frequency info to flags
	ASFLAGS += -DF_CPU=$(F_CPU)
	CPFLAGS += -DF_CPU=$(F_CPU)
	ifneq "$(MEM_START)""$(NORMAL_PROG_MEM_START)"
	CPFLAGS += -DBOOTLOADER_MEM_START=$(MEM_START)
	endif  
#this defines the aims of the make process
all:	$(TRG).elf $(TRG).hex $(TRG).eep $(TRG).ok 

#compile: instructions to create assembler and/or object files from C source
%o : %c 
	$(CC) -c $(CPFLAGS) -I$(INCDIR) $< -o $@

%s : %c
	$(CC) -S $(CPFLAGS) -I$(INCDIR) $< -o $@

#assemble: instructions to create object file from assembler files
%o : %s
	$(AS) -c $(ASFLAGS) -I$(INCDIR) $< -o $@

#link: instructions to create elf output file from object files
%elf: $(OBJ)
	$(CC) $(OBJ) $(LIB) $(LDFLAGS) -o $@

#create bin (ihex, srec) file from elf output file
%hex: %elf
	$(BIN) -O $(FORMAT) -R .eeprom $< $@
        ifneq "$(merge)"""
	$(MERGE) $(TRG).hex -intel $(merge) -intel -o $(TRG).hex -intel
        endif

%eep: %elf
	$(BIN) -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O $(FORMAT) $< $@


#If all other steps compile ok then echo "Errors: none".
#Necessary for AVR Studio to understand that everything went ok.
%ok:
	$(SIZE) $(TRG).elf
	$(SIZE)  $(TRG).hex
	$(SIZE)  $(TRG).eep
	@echo "Errors: none"
 
bin:	$(TRG).bin $(TRG).eep_bin 

%bin: %elf
	$(BIN) -O binary -R .eeprom $< $@   
	$(SIZE)  $(TRG).elf

%eep_bin: %elf
	$(BIN) -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -Obinary $< $@

srec:	$(TRG).srec

%srec: %elf
	$(BIN) -Osrec -R .eeprom $< $@
	$(SIZE)  $(TRG).srec

addboot:  $(TRG).hex
	  $(MERGE) $(TRG).hex -intel $(BOOTLOADER) -intel -o $(TRG)+$(BOOTLOADER) -intel

# Program the flash and/or Eeprom of the mcu.  
program:	
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) 

# Read device fuse bytes
fuse_read:	
	$(AVRDUDE) $(AVRDUDE_FUSE_FLAGS) $(AVRDUDE_READ_FUSE)

# Write device fuse bytes
fuse_write:	
	$(AVRDUDE) $(AVRDUDE_FUSE_FLAGS) $(AVRDUDE_WRITE_FUSE)


#make instruction to delete created files

clean:
	$(RM) $(OBJ)
#	$(RM) $(SRC:.c=.s)
	$(RM) $(SRC:.c=.lst)
	$(RM) $(TRG).map
	$(RM) $(TRG).elf
	$(RM) $(TRG).obj
	$(RM) $(TRG).a90
	$(RM) $(TRG).hex
	$(RM) $(TRG).bin
	$(RM) $(TRG).eep
	$(RM) $(TRG).eep_bin
	$(RM) $(TRG).srec
	$(RM) *.bak
	$(RM) *.log
	$(RM) $(TRG)+$(BOOTLOADER)
	
size:
	$(SIZE)  $(TRG).elf
#Use the below commented line instead of the above line if you use avr-sizex
#	$(SIZE)  --long --mcu $(MCU) $(TRG).elf


######################################################################################
#           U S E R   C O N F I G U R A T I O N  B L O C K  S T A R T                  
######################################################################################
###### dependecies, add any dependencies you need here                     ###########
###### IF A FILE FROM THE ONES LISTED BELOW IS EDITED A REBUILD IS FORCED  ########### 
######################################################################################

$(TRG).o   : $(TRG).c servo2ppm_settings.h 


######################################################################################
#           U S E R   C O N F I G U R A T I O N  B L O C K  E N D                  
######################################################################################






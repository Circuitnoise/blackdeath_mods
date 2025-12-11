# makefile, written by guido socher
MCU=atmega168
#MCU=at90s4433
CC=avr-gcc
#CEXTRA=-Wa,-adhlns=$(<:.c=.lst)
#EXTERNAL_RAM = -Wl,--defsym=__heap_start=0x801100,--defsym=__heap_end=0x80ffff
#EXTERNAL_RAM = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff
LDFLAGS  = -mmcu=${MCU} -Wl,-u, -lm
#LDFLAGS  = -mmcu=${MCU} -Wl,-u,vfprintf -lprintf_flt -lm
OBJCOPY=avr-objcopy
# optimize for size:
#CFLAGS=-g -mmcu=$(MCU) -Wall -Wstrict-prototypes -mcall-prologues ${CEXTRA}
CFLAGS=-g -mmcu=$(MCU) -Os
# AVR Header-Pfade angepasst
DEVICE = m168
AVRDUDE = avrdude -c usbasp -p $(DEVICE)
FUSEH = 0xdf
FUSEL = 0xf7


#-------------------
all: microbdinterp.hex
#-------------------
help: 
	@echo "Usage: make all|flash|read_firmware|rdfuses|fuse|clean"
	@echo ""
	@echo "Targets:"
	@echo "  all           - Build hexfile from source"
	@echo "  flash         - Flash hexfile to ATmega168"
	@echo "  read_firmware - Download firmware from ATmega168 to backup.hex"
	@echo "  rdfuses       - Read fuse bytes from ATmega168"
	@echo "  fuse          - Write default fuse bytes"
	@echo "  clean         - Remove build artifacts"
#-------------------
microbdinterp.hex : microbdinterp.out 
	$(OBJCOPY) -R .eeprom -O ihex microbdinterp.out microbdinterp.hex 
#microbdinterp.out : microbdinterp.o 
#	$(CC) $(CFLAGS) -o microbdinterp.out -Wl,-Map,microbdinterp.map microbdinterp.o 
microbdinterp.out : microbdinterp.o 
	$(CC) ${LDFLAGS} $(CFLAGS) -o microbdinterp.out  microbdinterp.o 


microbdinterp.o : microbdinterp.c 
	$(CC) $(CFLAGS) -Os -c microbdinterp.c

microbdinterp.elf: microbdinterp.o
	$(CC) ${LDFLAGS} $(CFLAGS) -o microbdinterp.elf microbdinterp.o

disasm:	microbdinterp.elf
	avr-objdump -d noise.elf

fuse:
	$(AVRDUDE) -F -U hfuse:w:$(FUSEH):m -U lfuse:w:$(FUSEL):m

flash: all
	$(AVRDUDE) -F -U flash:w:microbdinterp.hex:i

read_firmware:
	@echo "Reading firmware from ATmega168..."
	$(AVRDUDE) -U flash:r:backup.hex:i
	@echo "Firmware saved to backup.hex"

rdfuses:
	$(AVRDUDE) -U lfuse:r:-:b -U hfuse:r:-:b


#-------------------
clean:
	rm -f *.o *.map *.out *t.hex
#-------------------
 
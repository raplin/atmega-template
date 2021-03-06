#makefile from http://www.hookersandbooze.com/avr-gcc-makefile/ thx

# Change this to the main C file for your project
PRG            = main

#cpu target

#your cpu freq
HZ          = 16000000
#your program talks at this baud to the console (8n1)
CONSOLE_BAUD= 115200


# EITHER
# ardunio original via ftdi 
PROGRAMMER     = avrisp
PORT           = /dev/ttyUSB1
AVRDUDE_BAUD  = 19200
MCU_TARGET     = atmega168
AVRDUDE_TARGET = m168

# -- OR --
#arduino nano via usb
PROGRAMMER     = arduino
AVRDUDE_BAUD  = 57600
MCU_TARGET     = atmega328p
AVRDUDE_TARGET = m328p



OBJ            = $(PRG).o
OPTIMIZE       = -Os
DEFS           =
LIBS           =


# You should not have to change anything below here.

CC             = ./avr-gcc-col

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -DF_CPU=$(HZ) -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) \
-std=gnu99 -W -Wall  -Wundef  \
-funsigned-char -funsigned-bitfields -ffunction-sections -fpack-struct -fshort-enums \
-ffreestanding -Os -g -gdwarf-2 \
--combine -fwhole-program \
-fno-inline-small-functions -fno-split-wide-types -fno-tree-scev-cprop \
-Wl,--relax,--gc-sections \


override LDFLAGS       = -Wl,-Map,$(PRG).map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst text #eeprom

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf *.o $(PRG).elf *.eps *.png *.pdf *.bak *.hex *.bin *.srec
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec


ehex:  $(PRG)_eeprom.hex
#ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@

#%_eeprom.srec: %.elf
#	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@


# command to program chip (invoked by running "make install")
install:  $(PRG).hex
		avr-size $(PRG).elf
	    avrdude -V -b $(AVRDUDE_BAUD) -p $(AVRDUDE_TARGET) -c $(PROGRAMMER)  -P $(PORT) -F -U flash:w:$(PRG).hex

serialconsole:  
	    python serialconsole.py $(PORT) $(CONSOLE_BAUD) $(PRG).c

fuse:
	avrdude -p $(AVRDUDE_TARGET) -c $(PROGRAMMER) -P $(PORT) -v \
	-U lfuse:w:0xc6:m -U hfuse:w:0xd9:m 	

ddd: gdbinit
	ddd --debugger "avr-gdb -x $(GDBINITFILE)"

gdbserver: gdbinit
	simulavr --device $(MCU_TARGET) --gdbserver

gdbinit: $(GDBINITFILE)

$(GDBINITFILE): $(PRG).hex
	@echo "file $(PRG).elf" > $(GDBINITFILE)

	@echo "target remote localhost:1212" >> $(GDBINITFILE)
	@echo "load"                         >> $(GDBINITFILE)
	@echo "break main"                   >> $(GDBINITFILE)
	@echo
	@echo "Use 'avr-gdb -x $(GDBINITFILE)'"


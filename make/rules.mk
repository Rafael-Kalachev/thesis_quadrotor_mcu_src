

##
# C rules
##

%.o: %.c
	$(CC) $(C_FLAGS) -c -o $@  $^

#TODO change to use AS, but research first
%.o: %.s
	$(CC) $(C_FLAGS) -c -o $@  $^

##
# OBJECT rules
##

%.bin: %.elf
	$(OBJCOPY) -O binary $^  $@

%.hex: %.elf
	$(OBJCOPY) -O ihex $^  $@


##
# CTAGS
##

.PHONY: tags 

tags:
	$(CTAGS) $(CTAGS_FLAGS)



.PHONY: flash

flash: $(PROJECT)
	$(FLASH_CHIP) $(PROJECT) $(MCU_FLASH_MEMORY_START_ADDRESS)

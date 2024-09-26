
SSHPASS = /usr/bin/sshpass
REMOTE_IP ?= 10.0.0.1
REMOTE_PORT ?= 3333
REMOTE_USER ?= pi
REMOTE_PASSWORD ?= pi
TARGET_MCU_LOWER_CASE = $(shell echo $(TARGET_MCU) | tr A-Z a-z)

remote_flash_swd : check_dirty
	$(V0) $(MAKE) -j $(TARGET_ELF)
	$(SSHPASS) -p $(REMOTE_PASSWORD) \
		ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $(REMOTE_USER)@$(REMOTE_IP) \
			'sudo ln -sf /usr/share/openocd/scripts/target/$(TARGET_MCU_LOWER_CASE)x.cfg /opt/openocd/chip.cfg \
				&& sudo systemctl stop openocd.service \
				&& mkdir -p ~/indiflight/obj/main'
	rsync -a \
		--rsh "$(SSHPASS) -p $(REMOTE_PASSWORD) ssh -o StrictHostKeyChecking=no -l $(REMOTE_USER)" \
		--timeout=3 \
		$(TARGET_ELF) $(REMOTE_USER)@$(REMOTE_IP):/home/pi/indiflight/obj/main/
	$(SSHPASS) -p $(REMOTE_PASSWORD) \
		ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $(REMOTE_USER)@$(REMOTE_IP) \
			'openocd -f /opt/openocd/openocd.cfg \
			-c "program /home/pi/indiflight/$(TARGET_ELF) verify reset exit"'

remote_flash_dfu : check_dirty
	$(V0) $(MAKE) -j $(TARGET_DFU)
# temporarily stop ser2net to release tty device
	$(SSHPASS) -p $(REMOTE_PASSWORD) \
		ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $(REMOTE_USER)@$(REMOTE_IP) \
			'sudo systemctl stop ser2net.service && mkdir -p ~/indiflight/obj'
	sleep 1
# bring FC into DFU mode. Continue on failure, because maybe it's already in DFU mode
	-$(SSHPASS) -p $(REMOTE_PASSWORD) \
		ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $(REMOTE_USER)@$(REMOTE_IP) \
			"echo -n 'R' > /dev/ttyACM0"
	sleep 1
# update dfu file on raspberry 
	rsync -a \
		--rsh "$(SSHPASS) -p $(REMOTE_PASSWORD) ssh -o StrictHostKeyChecking=no -l $(REMOTE_USER)" \
		--timeout=3 \
		$(TARGET_DFU) $(REMOTE_USER)@$(REMOTE_IP):/home/pi/indiflight/obj/
# flash and restart ser2net
	$(SSHPASS) -p $(REMOTE_PASSWORD) \
		ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $(REMOTE_USER)@$(REMOTE_IP) \
			'dfu-util -a 0 -D /home/pi/indiflight/$(TARGET_DFU) -s :leave; \
				sudo systemctl start ser2net.service'

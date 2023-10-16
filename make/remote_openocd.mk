
SSHPASS = /usr/bin/sshpass
REMOTE_IP ?= 10.0.0.1
REMOTE_PORT ?= 3333
REMOTE_USER ?= pi
REMOTE_PASSWORD ?= pi
TARGET_MCU_LOWER_CASE = $(shell echo $(TARGET_MCU) | tr A-Z a-z)

remote_flash : $(TARGET_ELF)
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

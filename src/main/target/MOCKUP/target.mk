MOCKUP_TARGETS += $(TARGET)

TARGET_SRC = \
			mockup/main.c \
            drivers/accgyro/accgyro_fake.c \
            drivers/barometer/barometer_fake.c \
            drivers/compass/compass_fake.c \
            drivers/serial_tcp.c
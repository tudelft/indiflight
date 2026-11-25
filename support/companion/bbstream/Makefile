# Builds the companion-side decoder. The blackbox-tools sources are used as they are.
BBT   := ../../LogAnalysis/indiflight_log_tools/indiflight_log_tools/blackbox-tools/src
OBJ   ?= obj
CFLAGS ?= -O2 -Wall -Wextra -g
LIB   := parser.c tools.c platform.c stream.c decoders.c units.c blackbox_fielddefs.c

all: $(OBJ)/bbstream_cli $(OBJ)/blackbox_decode

$(OBJ)/bbstream_cli: bbstream.c bbstream_cli.c bbstream.h $(addprefix $(BBT)/,$(LIB))
	@mkdir -p $(OBJ)
	$(CC) $(CFLAGS) -I$(BBT) -o $@ bbstream.c bbstream_cli.c $(addprefix $(BBT)/,$(LIB)) -lm -lpthread

# the reference decoder, to compare against
$(OBJ)/blackbox_decode: $(addprefix $(BBT)/,$(LIB) blackbox_decode.c stats.c)
	@mkdir -p $(OBJ)
	$(CC) $(CFLAGS) -w -I$(BBT) -o $@ $^ -lm -lpthread

test: all
	python3 test_bbstream.py

clean:
	rm -rf $(OBJ)

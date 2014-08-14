CC = gcc
CFLAGS =  -Wall -pedantic 
LDFLAGS = -lm
BINARIES =  ubloxcfg

all: $(BINARIES) cleanobj

ubloxcfg: ubloxcfg.o serial.o
	$(CC) -o $@ $^ $(LDFLAGS)

cleanobj:
	$(RM) */*.o
	$(RM) *.o

clean: cleanobj
	$(RM) $(BINARIES)


TARGET = wiring_test

all: $(TARGET)

INCL= -I../../lcd -I../../wiring -I../../twi -I../../esc

CC= avr-gcc -Os -FS_CPU=1000000UL -mmcu=attiny85 $(INCL) -c
CL= avr-gcc -mmcu=attiny85

$(TARGET): wiring_test.o wiring.o
	$(CL) $^ -o $@

wiring_test.o: test.cpp ../../wiring/wiring.h
	$(CC) -o $@ $<

wiring.o: ../../wiring/wiring.cpp ../../wiring/wiring.h
	$(CC) -o $@ $<

clean:
	rm *.o

download: $(TARGET)
	./../../../mfile_upload $(TARGET)

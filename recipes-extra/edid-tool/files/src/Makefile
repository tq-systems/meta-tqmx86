CFLAGS += -Wall

TARGET = edid-tool
OBJS = edid-tool.o i2cbusses.o
HEADERS = i2cbusses.h

all: $(TARGET)

$(TARGET): $(OBJS)
$(OBJS): $(HEADERS)

clean:
	rm -f $(TARGET) *.o

.PHONY: clean

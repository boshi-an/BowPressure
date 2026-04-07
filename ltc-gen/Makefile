CXX      := g++
CXXFLAGS := -O2 -Wall -Wextra -std=c++17
LDFLAGS  := -lltc -lasound

TARGET   := ltc_stream
SRC      := ltc_stream.cpp

.PHONY: all clean install uninstall

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(TARGET)

uninstall:
	rm -f /usr/local/bin/$(TARGET)

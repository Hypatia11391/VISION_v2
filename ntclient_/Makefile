:qCXX := g++

NATIVE := $(VIRTUAL_ENV)/lib/python3.13/site-packages/native
WPI := $(NATIVE)/ntcore

CXXFLAGS := -std=c++20 -O2 \
    -I$(NATIVE)/ntcore/include \
    -I$(NATIVE)/wpiutil/include \
    -I$(NATIVE)/wpinet/include \
    -I$(NATIVE)/wpimath/include \
    -I$(NATIVE)/wpilib/include \
    -I$(NATIVE)/wpihal/include

LDFLAGS := \
    -L$(NATIVE)/ntcore/lib \
    -L$(NATIVE)/wpinet/lib \
    -L$(NATIVE)/wpiutil/lib \
    -Wl,-rpath,$(NATIVE)/ntcore/lib \
    -Wl,-rpath,$(NATIVE)/wpinet/lib \
    -Wl,-rpath,$(NATIVE)/wpiutil/lib

LDLIBS := -lntcore -lwpinet -lwpiutil

TARGET := ntclient
SRC := ntclient.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET)

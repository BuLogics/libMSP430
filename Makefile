CXX:= g++

DEFINES := -DUNIX

CXXFLAGS := -fPIC
BOOST_DIR := $(PWD)/../boost_1_51_0
#CXXFLAGS := -fPIC -m32

ifdef DEBUG
CXXFLAGS += -g -O0
else
CXXFLAGS += -Os
DEFINES += -DNDEBUG
endif

INCLUDES := \
	-I./Bios/include \
	-I./HalObjectDb/include \
	-I./DLL430_v3/include \
	-I./DLL430_v3/include/DLL430 \
	-I./DLL430_v3/src \
	-I./DLL430_v3/src/TI/DLL430

LIBDIRS :=


ifdef BOOST_DIR
INCLUDES += -I$(BOOST_DIR)
LIBDIRS += -L$(BOOST_DIR)/stage/lib
endif

LIBS := -lboost_thread -lboost_filesystem -lboost_date_time -lboost_system

SRC := \
	./HalObjectDb/src/HalObjectDb.cpp \
	./DLL430_v3/src/DLL430v3_OS_capi.cpp \
	./DLL430_v3/src/DLL430_plugin.cpp \
	./DLL430_v3/src/DLL430_OldApiV3.cpp \
	./DLL430_v3/src/TI/DLL430/EEM/CycleCounter.cpp \
	$(wildcard ./DLL430_v3/src/TI/DLL430/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/EM/BreakpointManager/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/EM/CycleCounter/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/EM/EemRegisters/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/EM/EmulationManager/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/EM/Exceptions/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/EM/Sequencer/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/EM/StateStorage430/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/EM/Trace/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/EM/Trigger/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/EM/TriggerCondition/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/EM/TriggerManager/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/EM/VariableWatch/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/logging/*.cpp) \
	$(wildcard ./DLL430_v3/src/TI/DLL430/TemplateDeviceDb/*.cpp)
	
OBJS := $(patsubst %.cpp, %.o, $(SRC))

OUTPUT := libmsp430.so

all: $(OBJS)
#	$(CXX) $(CXXFLAGS) -shared -Wl,-soname,$(OUTPUT) -o $(OUTPUT) $(OBJS) $(LIBDIRS) -Wl,-Bstatic $(LIBS) -Wl,-Bdynamic -lpthread
	$(CXX) $(CXXFLAGS) -shared -Wl,-soname,$(OUTPUT) -o $(OUTPUT) $(OBJS) $(LIBDIRS) -Wl,$(LIBS) -Wl,-Bdynamic -lpthread

%.o: %.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(INCLUDES) $(DEFINES)

install:
	cp $(OUTPUT) /usr/lib/

clean:
	@for i in $(OBJS); do rm -f $$i; done
	@rm -f build.log

 

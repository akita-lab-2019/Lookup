mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS += \
	GuageManager.o \
	Recorder.o \
	LineMonitor.o \
	TailController.o \
	BalancerCpp.o \
	Odometer.o \
	TailWalker.o \
	PID.o \
	LogManager.o \
	Section.o \
	SectionTracer.o \
	BluetoothManager.o \
	StartManager.o \
	LineTracer.o \
	Seesaw.o \
	Lookup.o

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIR += $(mkfile_path)Control\
            $(mkfile_path)GaugeManagement\
            $(mkfile_path)Log\
            $(mkfile_path)RunManagement\
            $(mkfile_path)Start\
            $(mkfile_path)Tail\
            $(mkfile_path)Seesaw\
            $(mkfile_path)Lookup

INCLUDES += -I$(mkfile_path)Control\
            -I$(mkfile_path)GaugeManagement\
            -I$(mkfile_path)Log\
            -I$(mkfile_path)RunManagement\
            -I$(mkfile_path)Start\
            -I$(mkfile_path)Tail\
            -I$(mkfile_path)Seesaw\
            -I$(mkfile_path)Lookup

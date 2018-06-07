.PHONY: all

all: lndw_ballsorter;

lndw_ballsorter : hello_pixy.cpp robot.cpp gpio.cpp
	g++ -o lndw_ballsorter hello_pixy.cpp robot.cpp gpio.cpp -llibftdi1/ftdi -llibusb -fpermissive

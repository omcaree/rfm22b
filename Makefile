CXXFLAGS=-I./include

all: rfm22b_test

rfm22b_test : rfm22b_test.cpp src/rfm22b.o src/spi.o
	g++ $(CXXFLAGS) rfm22b_test.cpp src/rfm22b.o src/spi.o -o rfm22b_test
	
src/rfm22b.o: src/rfm22b.cpp include/rfm22b.h
src/spi.o: src/spi.cpp include/spi.h

clean:
	rm rfm22b_test src/*.o
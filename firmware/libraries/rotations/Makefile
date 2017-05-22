run: main
	./main
main: main.cpp rotation.o vectors.o
%.o: %.cpp
	g++ -c -g $<
clean:
	rm -rf *.o
	rm -rf main


FLAGS = -std=c++11 -g
OBJS = rotation.o vectors.o
CC = g++
run: main
	./main
main: main.cpp rotation.o vectors.o
	$(CC) $(FLAGS) $(OBJS) -o main $< 
%.o: %.cpp
	$(CC) $(FLAGS) -c $<
main.cpp:
	ln -s main.cpp.dontlookatmearduino main.cpp
clean:
	rm -rf *.o
	rm -rf main
	rm main.cpp


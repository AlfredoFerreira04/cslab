CC = gcc
CFLAGS = -g -Wall -Wextra
LDFLAGS = -lpaho-mqtt3cs -lcjson -z noexecstack

all: server

server: srv.o asm.o
	$(CC) srv.o asm.o $(LDFLAGS) -o server

srv.o: srv.c asm.h
	$(CC) $(CFLAGS) -c srv.c -o srv.o

asm.o: asm.s
	$(CC) $(CFLAGS) -c asm.s -o asm.o

clean:
	rm -f server *.o *.log

run: server
	./server

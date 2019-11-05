test: test1.c light/light-state.c light/user-interface.c light/light-impl.c system.c
	gcc test1.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c -lcmockery -o test1 -g -Wall
	./test1

clean:
	rm -f test1

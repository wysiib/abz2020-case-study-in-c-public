.Phony: all
all: clean test main cbmc_light

.PHONY: test_sequences test_els

test: test_sequences test_els

test_sequences: test1.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c
	gcc test1.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c -lcmockery -o test1 -g -Wall
	./test1

test_els: test_els.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c
	gcc test_els.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c -lcmockery -o test_els -g -Wall
	./test_els

main: main.c light/light-state.c light/user-interface.c light/light-impl.c system.c
	gcc light/light-state.c light/user-interface.c light/light-impl.c system.c -lcmockery -o main -g -Wall

clean:
	rm -f test1

cbmc_light:
	cbmc light/*.c --function light_loop

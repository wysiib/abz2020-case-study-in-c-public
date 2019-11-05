.PHONY: test_sequences test_els

test: test_sequences test_els

test_sequences: test1.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c
	gcc test1.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c -lcmockery -o test1 -g -Wall
	./test1

test_els: test_els.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c
	gcc test_els.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c -lcmockery -o test_els -g -Wall
	./test_els

clean:
	rm -f test1

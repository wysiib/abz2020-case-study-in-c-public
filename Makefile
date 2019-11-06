.PHONY: test_sequences test_els test_scs

test: test_sequences test_els test_scs

test_sequences: test1.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c
	gcc test1.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c -lcmockery -o test1 -g -Wall
	./test1

test_els: test_els.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c
	@echo "Running exterior light tests"
	gcc test_els.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c -lcmockery -o test_els -g -Wall
	./test_els

test_scs: test_scs.c test_common.c system.c light/light-state.c cruise-control/scs-impl.c cruise-control/scs-state.c
	@echo "Running speed control tests"
	gcc test_scs.c test_common.c system.c light/light-state.c cruise-control/scs-impl.c cruise-control/scs-state.c -lcmockery -o test_scs -g -Wall
	./test_scs

main: main.c light/light-state.c light/user-interface.c light/light-impl.c system.c
	gcc light/light-state.c light/user-interface.c light/light-impl.c system.c -lcmockery -o main -g -Wall

clean:
	rm -f test1

cbmc_light:
	cbmc light/*.c --function light_loop

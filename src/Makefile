.PHONY: test_sequences test_els test_scs

test: test_sequences test_els test_scs

test_sequences: test1.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c
	gcc test1.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c -lcmockery -o test1 -g -Wall
	./test1

test_els: test_els.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c
	@echo "Running exterior light tests"
	gcc test_els.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c -lcmockery -o test_els -g -Wall
	./test_els

vis_els: test_els.c test_common.c light/light-state.c light/user-interface.c light/light-impl.c system.c 
	@echo "light visualize"
	gcc -g -D vis test_els.c test_common.c light/light-state.c light/user-interface.c  light/light-impl.c system.c -lcmockery -o test_els -g -Wall
	./test_els


.PHONY: build_test_scs
build_test_scs: test_scs.c test_common.c system.c light/light-state.c cruise-control/scs-impl.c cruise-control/scs-state.c cruise-control/user-interface.c
	gcc test_scs.c test_common.c system.c light/light-state.c cruise-control/scs-impl.c cruise-control/scs-state.c cruise-control/user-interface.c -lcmockery -o test_scs -g -Wall

test_scs: build_test_scs
	@echo "Running speed control tests"
	./test_scs

main: main.c light/light-state.c light/user-interface.c light/light-impl.c cruise-control/scs-impl.c cruise-control/scs-state.c cruise-control/user-interface.c system.c main.c test_common.c
	gcc light/light-state.c light/user-interface.c light/light-impl.c cruise-control/scs-impl.c cruise-control/scs-state.c cruise-control/user-interface.c system.c main.c test_common.c -o main -lcmockery -g -Wall -lpthread

dll: light/light-state.c light/user-interface.c light/light-impl.c cruise-control/scs-impl.c cruise-control/scs-state.c cruise-control/user-interface.c system.c external_sensor_driver.c
	gcc -fpic -shared light/light-state.c light/user-interface.c light/light-impl.c cruise-control/scs-impl.c cruise-control/scs-state.c cruise-control/user-interface.c system.c external_sensor_driver.c -o libabz.dll

clean:
	rm -f test1
	rm -f main
	rm -rf *.dSYM

cbmc_light:
	cbmc light/*.c --unwind 20 --unwinding-assertions --function light_loop --trace --beautify

cbmc_light_step:
	cbmc light/*.c system.c --unwind 20 --unwinding-assertions --function light_do_step  #--trace --beautify

.Phony: all
all: clean test

misra_check:
	cppcheck --dump common cruise-control light
	python3 ../scripts/misra.py --rule-texts=../scripts/MISRAC2012_Rules.txt cruise-control/*.dump light/*.dump

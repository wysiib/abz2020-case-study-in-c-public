test: test1.c
	gcc test1.c -lcmockery -o test1

clean:
	rm -f test1

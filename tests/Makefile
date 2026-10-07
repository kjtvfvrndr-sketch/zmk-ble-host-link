# Runs the search and the decisions around it on the build machine:
#   search_test     the search against simulated hosts
#   decisions_test  stored results, refusals, drops, ZMK Studio -- as tables
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O1 -g

search_test: search_test.c ../src/search.c ../src/search.h
	$(CC) $(CFLAGS) -o $@ search_test.c ../src/search.c

decisions_test: decisions_test.c ../src/decisions.c ../src/decisions.h ../src/search.c ../src/search.h
	$(CC) $(CFLAGS) -o $@ decisions_test.c ../src/decisions.c ../src/search.c

.PHONY: run clean
run: search_test decisions_test
	./search_test
	./decisions_test

clean:
	rm -f search_test decisions_test

.DEFAULT_GOAL := run

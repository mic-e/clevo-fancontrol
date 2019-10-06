.PHONY: all
all: clevo-fancontrol

clevo-fancontrol: clevo-fancontrol.c Makefile
	gcc -Wall -Wextra -pedantic clevo-fancontrol.c -o clevo-fancontrol -DPRINT_STATE=0

.PHONY: install
install: clevo-fancontrol
	install -m 755 clevo-fancontrol /usr/local/bin/
	install -m 644 clevo-fancontrol.service /lib/systemd/system

.PHONY: clean
clean:
	rm -f clevo-fancontrol

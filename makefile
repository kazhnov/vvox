DEBUG = -O0 -g -fsanitize=address
RELEASE = -O3
MODE = $(DEBUG)

NAME = vvox
INCLUDE = include
OBJECTS := $(wildcard $(INCLUDE)/*/build/*.o)

subs:
	git submodule update --init --recursive
	git submodule foreach git pull origin main
	(cd $(INCLUDE)/vmesh/  && make build -B)
	(cd $(INCLUDE)/vshape/ && make build -B)


build: $(NAME).c
	mkdir -p build lib
	cc -c $(NAME).c -o build/$(NAME).o $(MODE)
	ar rcs lib/lib$(NAME).a build/$(NAME).o $(OBJECTS)

test: test.c build
	cc test.c -o build/test -L./lib -lm -l$(NAME) $(MODE) 
	build/test

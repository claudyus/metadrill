all:
	gcc -o metadrill -ggdb -Wall -O0 metadrill.c -lm -lSDL -lSDL_ttf

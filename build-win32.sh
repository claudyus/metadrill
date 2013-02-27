
# Download and extract in this directory:
# SDL-devel-1.2.13-mingw32.tar.gz from http://www.libsdl.org/download-1.2.php
# SDL_ttf-devel-2.0.9-VC8.zip from http://www.libsdl.org/projects/SDL_ttf/

set -v
mkdir -p win32_bin

SDL_DIR=$PWD/SDL-1.2.13
SDL_TTF_DIR=$PWD/SDL_ttf-2.0.9

rm -rf $SDL_TTF_DIR/include/SDL
mkdir $SDL_TTF_DIR/include/SDL
cp $SDL_TTF_DIR/include/*.h $SDL_TTF_DIR/include/SDL/

gcc -o win32_bin/metadrill -mwindows -ggdb -Wall -O0 -I$SDL_DIR/include -I$SDL_DIR/include/SDL -L$SDL_DIR/lib -I$SDL_TTF_DIR/include -L$SDL_TTF_DIR/lib metadrill.c -lm -lmingw32 -lSDLmain -lSDL -lSDL_ttf -mwindows
gawk '{ print $0 "\r"; }' README > win32_bin/README.txt
cp $SDL_DIR/bin/*.dll $SDL_TTF_DIR/lib/*.dll win32_bin/
cp font.ttf win32_bin/


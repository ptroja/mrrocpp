#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>

using namespace std;

const int MAX_DEPTH= 40;
const int MAX_MOVE_LENGTH= 25;
const int MAX_MOVES_PER_POSITION= 25;

const int INFINITY= 10000;

const int WKING =0, BKING=1;
const int WMAN  =2, BMAN=3;
const int EMPTY=4,  BORDER=5;

#include "player.cpp"

#include "moves.cpp"
TMoveGenerator TMoveGenerator;

#include "eval.cpp"

#include "search.cpp"
TSearcher TSearcher;

#include "game.cpp"

main(){
	TGame Game;
	Game.Play();
	return 0;
}

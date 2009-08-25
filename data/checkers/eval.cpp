const int CHECKER  =	 100; // a checkers' worth
const int KING	  =	 130; // a kings' worth
const int BACKRANK =	  10;

int Evaluation(int *Board, TPlayer Player){
  int Score=0 ;
  int NWKings=0, NBKings=0, NWMen=0, NBMen=0;

  for( register int i=10 ; i<45 ; ++i ){
	 if(IsPiece(Board[i])){
		int Piece= Board[i];

		switch(Piece){
		  case WMAN:
				NWMen++;
				break;

		  case BMAN:
				NBMen++;
				break;

		  case WKING:
				NWKings++;
				break;

		  case BKING:
				NBKings++;
				break;
		}
	 }
  }

  if(Board[10]==WMAN&&Board[12]==WMAN&&NBMen>1)
	 Score-=BACKRANK;

  if(Board[44]==BMAN&&Board[42]==BMAN&&NWMen>1)
	 Score+=BACKRANK;

  int MaterialBlack= NBKings*KING+NBMen*CHECKER;
  int MaterialWhite= NWKings*KING+NWMen*CHECKER;

  //hints to exchange pieces if ahead in material
  Score+= ((MaterialBlack-MaterialWhite)*200)
				/(MaterialBlack+MaterialWhite);

  Score+=MaterialBlack-MaterialWhite;
  return Player==BLACK?Score:-Score;
}

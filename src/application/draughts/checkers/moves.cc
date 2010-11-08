class TMoveList{
	public:
	int **List;
	int NMoves;
	bool MustTake;
	TMoveList();
	~TMoveList();
};

TMoveList::TMoveList(){
	List= new int*[MAX_MOVES_PER_POSITION];
	for(int i=0;i<MAX_MOVES_PER_POSITION;++i)
		List[i]=new int[MAX_MOVE_LENGTH];
}

TMoveList::~TMoveList(){
	for(int i=0;i<MAX_MOVES_PER_POSITION;++i)
		delete []List[i];
	delete []List;
}

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

class TMoveGenerator{
		int JumpValue[6];
		int PiecePath[64];
		int NSteps;
		int *AuxBoard;
		TPlayer AuxPlayer;
		TMoveList *AuxMoveList;
		void SaveMove();
		void MakeMoveListAux( int Square, int Depth);
	public:
		TMoveGenerator();
		void MakeMoveList( int *Board, TPlayer Player, TMoveList *MoveList);
		void InitBoard(int *Board);
		void MakeMove(int Move[],int *Board);
		bool NoMovesLeft( int *Board, TPlayer Player);
};


TMoveGenerator::TMoveGenerator(){
	JumpValue[0]=4;
	JumpValue[1]=5;
	JumpValue[2]=-5;
	JumpValue[3]=-4;
	JumpValue[4]=4;
	JumpValue[5]=5;
	AuxPlayer=BLACK;
}

void TMoveGenerator::SaveMove(){
	register int *Source= PiecePath;
	register int *Dest= AuxMoveList->List[AuxMoveList->NMoves++];
	Source[NSteps]=0; // 0 means end of move
	
	do{
		*(Dest++)=*Source;
	}while( *(Source++) );
	
	return;
}

void TMoveGenerator::MakeMoveListAux( int Square, int Depth){
	PiecePath[NSteps++]= Square;
	bool MustSaveMove= true;
	int i= (AuxBoard[Square]%2)*2 ;
	int EndLoop= i + ( AuxBoard[Square]>BKING?2:4 );

	for( ; i<EndLoop ; ++i){
		int Dx= JumpValue[i];
		if( AuxBoard[Square+Dx]==EMPTY && !AuxMoveList->MustTake ){
			if( NSteps==1 ){
				PiecePath[NSteps++]= Square+Dx;
				SaveMove();
				--NSteps;
			}
		}else if( IsPiece(AuxBoard[Square+Dx])
				&& BelongsTo(AuxBoard[Square+Dx],Other(AuxPlayer)) &&
				AuxBoard[Square+2*Dx]==EMPTY){
			if(!AuxMoveList->MustTake){
				AuxMoveList->MustTake= true;
				AuxMoveList->NMoves= 0;
			}
			
			PiecePath[NSteps++]= Square+Dx;

	      AuxBoard[Square+2*Dx]= AuxBoard[Square];
	      int Save= AuxBoard[Square+Dx];
	      AuxBoard[Square+Dx]= AuxBoard[Square]= EMPTY;
	      MakeMoveListAux(Square+2*Dx, Depth+1);
	      AuxBoard[Square]= AuxBoard[Square+2*Dx];
	      AuxBoard[Square+Dx]= Save;
	      AuxBoard[Square+2*Dx]= EMPTY;

	      NSteps--;
	      MustSaveMove= false;
		}//elseif
	}//for
	
	if( Depth && MustSaveMove )
		SaveMove();
		
	--NSteps;
	return;
}

void TMoveGenerator::MakeMoveList( int *Board, TPlayer Player, TMoveList *MoveList){
	MoveList->NMoves= NSteps= 0;
	MoveList->MustTake= false;
	AuxBoard= Board;
	AuxPlayer= Player;
	AuxMoveList= MoveList;
	for(register int i=10 ; i<45 ; ++i)
		if( IsPiece(Board[i]) && BelongsTo(Board[i],Player) )
			MakeMoveListAux(i,0);
}

void TMoveGenerator::InitBoard(int *Board){
	for(int i=0;i<55;++i)
		Board[i]=BORDER;
	for(int i=10;i<45;++i)
		Board[i]=EMPTY;

	for(int i=10;i<23;++i)
		Board[i]=WMAN;

	for(int i=32;i<45;++i)
		Board[i]=BMAN;

	Board[18]= Board[27]= Board[36]= BORDER;
}

void TMoveGenerator::MakeMove(int Move[],int *Board){
	int Piece=Board[*Move];
	int *Last;
	
	do{
		Last= Move;
		Board[*(Move++)]=EMPTY;
	}while(*Move);
	
	if( (*Last<14 || *Last>40) &&Piece>BKING ) //Check if man becomes a King
		Piece-=2;
		
	Board[*Last]=Piece;
}

bool TMoveGenerator::NoMovesLeft( int *Board, TPlayer Player){
	for( int Sq=0 ; Sq<45 ; ++Sq )
		if( IsPiece(Board[Sq]) && BelongsTo(Board[Sq],Player)){
			int i= (Board[Sq]%2)*2;
			int EndLoop= i + ( Board[Sq]>BKING?2:4 );
			
			for( ; i<EndLoop ; ++i){
				int Dx= JumpValue[i];
				if( Board[Sq+Dx]==EMPTY ){
					return false;  //can move
				}
				
				if( IsPiece(Board[Sq+Dx])&&BelongsTo(Board[Sq+Dx],Other(Player))&&
					Board[Sq+2*Dx]==EMPTY){
					return false; //can take a piece so at least a valid move
				}//elseif
			}//for
		}//if and for
		
	return true;  //no moves left
}

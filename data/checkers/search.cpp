struct TMoveInfo{
	int Evaluation;
	int *Move;
};


class TSearcher{
		  TMoveList *MoveListArray;
		  int **SavingArray;
		  int MaxDepth;
		  int SearchDepth;
		  char MoveInfoStr[128];
		  char MoveInfoSend[6];
		  long NEvaluated;
		  long NNodes;
	public:
		  TSearcher();
		 ~TSearcher();
		  void SetSearchDepth(int _SearchDepth){ SearchDepth= _SearchDepth; }
		  int AlphaBeta(int *Board, int Depth, TPlayer Player, int Lowest, int Highest,
		                int RealDepth);
		  TMoveInfo AlphaBeta0(int *Board, int Depth, TPlayer Player);

		  void GetComputerMove(int *Board, TPlayer Player, int *Move);
		  void MakeMove(int *Board, int *Move, int *Save);
		  void UndoMove(int *Board, int *Move, int *Save);
		  void CopyMove( int *Dest, int *Src);
		  bool IsOnlyMove(TMoveInfo &Info, int *Board, TPlayer Player);
		  void PrintMoveInfo(const TMoveInfo &Info, bool OnlyMove);
		  bool IsWinOrLoseMove(int e){ return abs(e)>INFINITY-500; }
		  char *GetMoveInfoStr(){ return MoveInfoStr; }
		  char *GetMoveInfoSend(){return MoveInfoSend;}

		  char* MoveToStr(int *Move,char *Str);
		  int SquareToNotation(int Sq); //converts internal representation to notation

 };

char *TSearcher::MoveToStr(int *Move,char *Str){
	int First= *Move;
	int i=0;
	while(*Move){
		++Move;
		++i;
	}
	sprintf(Str,"%d%c%d",SquareToNotation(First),i==2?'-':'x',SquareToNotation(*(--Move)));
	return Str;
}


int TSearcher::SquareToNotation(int Sq){
	int n= 45-Sq;
	if(Sq<36)n--;
	if(Sq<27)n--;
	if(Sq<18)n--;
	return n;
}


TSearcher::TSearcher(){
	MoveListArray= new TMoveList[MAX_DEPTH];
	SavingArray= new int*[MAX_DEPTH];
	for(int i=0;i<MAX_DEPTH;++i)
		SavingArray[i]= new int[MAX_MOVE_LENGTH];
}

TSearcher::~TSearcher(){
	delete[] MoveListArray;
	for(int i=0;i<MAX_DEPTH;++i)
		delete[] SavingArray[i];
	delete SavingArray;
}


void TSearcher::MakeMove(int *Board, int *Move, int *Save){
	int Piece= Board[*Move];
	do{
		*(Save++)= Board[*Move];
		Board[*(Move++)]= EMPTY;
	}while(*Move);
	int LastSquare= *(Move-1);
	if((LastSquare<14||LastSquare>40)&&Piece>BKING)
		Piece-=2; //man becomes a king
	Board[LastSquare]= Piece;
}

void TSearcher::UndoMove(int *Board, int *Move, int *Save){
	do{
		Board[*(Move++)]= *(Save++);
	}while(*Move);
}

void TSearcher::CopyMove( int *Dest, int *Src){
	do{
		*(Dest++)=*(Src++);
	}while(*Src);
	*Dest=0;
}

int TSearcher::AlphaBeta(int *Board, int Depth, TPlayer Player, int Lowest, int Highest,
              int RealDepth){
	NNodes++;
	if(RealDepth>MaxDepth)
	MaxDepth=RealDepth;

	TMoveList &MoveList= MoveListArray[RealDepth];
	TMoveGenerator.MakeMoveList(Board, Player, &MoveList);
	if(!MoveList.NMoves)
		return -INFINITY+RealDepth-1;

	/*
	Quiescence search:
	Expands capturing moves
	*/
	if(Depth<1&&MoveList.MustTake)
		Depth+= 1;

	if(Depth<1||RealDepth>=MAX_DEPTH-1){
		++NEvaluated;
		return Evaluation(Board,Player);
	}

	int Best=-INFINITY;
	for(int i=0;i<MoveList.NMoves&&Best<Highest;++i){
		MakeMove(Board, MoveList.List[i], SavingArray[RealDepth]);
		int Score=-AlphaBeta(Board, Depth-1,Other(Player),-Highest,-Lowest,RealDepth+1);
		UndoMove(Board, MoveList.List[i],SavingArray[RealDepth]);
		if(Score>Best){
			Best= Score;
			if(Score>Lowest)
				Lowest= Best;
		}
	}
   return Best;
}


TMoveInfo TSearcher::AlphaBeta0(int *Board, int Depth, TPlayer Player){
	int Lowest= -INFINITY;
	int Highest= INFINITY;

	TMoveList &MoveList= MoveListArray[0];
	TMoveGenerator.MakeMoveList(Board, Player, &MoveList);

	int Best=-INFINITY;
	int BestPos=0;
	for(int i=0;i<MoveList.NMoves&&Best<Highest;++i){
		MakeMove(Board, MoveList.List[i],SavingArray[0]);
		int Score= -AlphaBeta(Board, Depth-1, Other(Player), -Highest, -Lowest,1);
		UndoMove(Board, MoveList.List[i],SavingArray[0]);
		if(Score>Best){
			Best= Score;
			BestPos= i;
			if(Score>Lowest)
				Lowest= Best;
		}
	}
	
	TMoveInfo Temp;
	Temp.Evaluation= Best;
	Temp.Move= MoveList.List[BestPos];
	return Temp;
}

void TSearcher::PrintMoveInfo(const TMoveInfo &Info, bool OnlyMove){
	MoveToStr(Info.Move, MoveInfoStr);
	MoveToStr(Info.Move, MoveInfoSend);
	if(OnlyMove){
		strcat(MoveInfoStr," only move");
	}else{
		strcat(MoveInfoStr,"  Score=");
		char Aux[128];
		if(IsWinOrLoseMove(Info.Evaluation)){
			if(Info.Evaluation<0)
				sprintf(Aux, "lose in %d", INFINITY+Info.Evaluation);
			else
				sprintf(Aux, "win in %d", INFINITY-Info.Evaluation);
		}else{
			sprintf(Aux,"%d",Info.Evaluation);
		}
		
		strcat(MoveInfoStr,Aux);
		sprintf(Aux,"  Depth:%d/%d  ",SearchDepth,MaxDepth);
		strcat(MoveInfoStr,Aux);
		sprintf(Aux," Evals:%ld  Nodes:%ld", NEvaluated,NNodes);
		strcat(MoveInfoStr,Aux);
	}
}

bool TSearcher::IsOnlyMove(TMoveInfo &Info, int *Board, TPlayer Player){
	TMoveList &MoveList= MoveListArray[0];
	TMoveGenerator.MakeMoveList(Board,Player,&MoveList);
	if(MoveList.NMoves!=1)
		return false;
	Info.Move= MoveList.List[0];
	return true;
}

void TSearcher::GetComputerMove(int *Board, TPlayer Player, int *Move){
	TMoveInfo MoveInfo;
	MaxDepth=0;
	NEvaluated=0;
	NNodes=0;
	if(IsOnlyMove(MoveInfo, Board, Player)){
		PrintMoveInfo(MoveInfo,true);
	}else{
		MoveInfo= AlphaBeta0(Board,SearchDepth,Player);
		PrintMoveInfo(MoveInfo,false);
	}
	CopyMove(Move, MoveInfo.Move);
}

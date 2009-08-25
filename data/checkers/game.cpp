using namespace std;
/*print text representation of pawns*/
char PtC(int Piece){
	switch(Piece){
		case WKING: return 'W';
		case BKING: return 'B';
		case WMAN:  return 'w';
		case BMAN:  return 'b';
		default:	 return '-';
	}
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*print current board*/
void ShowBoard(int *Board){
	printf("	 White\n");
	printf("  %c  %c  %c  %c     32  31  30  29    0   1   2   3\n",PtC(Board[10]),PtC(Board[11]),PtC(Board[12]),PtC(Board[13]));
	printf("%c  %c  %c  %c     28  27  26  25    4   5   6   7 \n",PtC(Board[14]),PtC(Board[15]),PtC(Board[16]),PtC(Board[17]));
	printf("  %c  %c  %c  %c     24  23  22  21    8   9   10  11\n",PtC(Board[19]),PtC(Board[20]),PtC(Board[21]),PtC(Board[22]));
	printf("%c  %c  %c  %c     20  19  18  17    12  13  14  15\n",PtC(Board[23]),PtC(Board[24]),PtC(Board[25]),PtC(Board[26]));
	printf("  %c  %c  %c  %c     16  15  14  13    16  17  18  19\n",PtC(Board[28]),PtC(Board[29]),PtC(Board[30]),PtC(Board[31]));
	printf("%c  %c  %c  %c     12  11  10  9     20  21  22  23\n",PtC(Board[32]),PtC(Board[33]),PtC(Board[34]),PtC(Board[35]));
	printf("  %c  %c  %c  %c     8   7   6   5     24  25  26  27\n",PtC(Board[37]),PtC(Board[38]),PtC(Board[39]),PtC(Board[40]));
	printf("%c  %c  %c  %c     4   3   2   1     28  29  30  31\n",PtC(Board[41]),PtC(Board[42]),PtC(Board[43]),PtC(Board[44]));
	printf("	 Black\n");
}

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

enum TPlayerType{HUMAN,COMPUTER};

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

struct game_board_struct{
	char board[32];
	char player;
};

struct result_struct{
	char move[25];
	char status;
};

class TGame{
		int Board[55];
		TPlayer Turn;
		int NMove;
		char PlayerName[2][12];
		TPlayerType PlayerType[2];
		int SarchDepth[2];
	public:
		TGame();
		void Options();
		void Play();
};

const int	AI_NORMAL_MOVE=0;
const int	AI_COMPUTER_WON=1;
const int	AI_HUMAN_WON=2;

/*---------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*constructor*/
TGame::TGame(){
	TMoveGenerator.InitBoard(Board);
	NMove=0;
	Turn= BLACK;
	strcpy(PlayerName[WHITE],"WHITE");
	strcpy(PlayerName[BLACK],"BLACK");
	PlayerType[BLACK]= HUMAN;
	PlayerType[WHITE]= COMPUTER;
	SarchDepth[BLACK]=SarchDepth[WHITE]= 10;
}

/*---------------------------------------------------------------------------------------------------------------------------------------------------------*/
/**/
void TGame::Play(){
	bool EndOfGame= false;
	int Move[MAX_MOVE_LENGTH];
	char Aux[256];
	
	//network
	int listenSocket, connectSocket;
	unsigned short int listenPort=10050;
	socklen_t clientAddressLength;
	struct sockaddr_in clientAddress, serverAddress;
	struct game_board_struct gameBoard;
	struct result_struct result;

	listenSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (listenSocket < 0) {
		printf("cannot create listen socket");
		exit(1);
	}
	
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddress.sin_port = htons(listenPort);
	
	if (bind(listenSocket,(struct sockaddr *) &serverAddress,sizeof(serverAddress)) < 0) {
		printf("cannot bind socket");
		exit(1);
	}

	// Wait for connections from clients.
	listen(listenSocket, 5);
	
	while(1){
		printf("Waiting for TCP connection on port %d...\n",listenPort);
		
		// Accept a connection with a client that is requesting one.
		clientAddressLength = sizeof(clientAddress);
		connectSocket = accept(listenSocket,(struct sockaddr *) &clientAddress, &clientAddressLength);

		if (connectSocket < 0) {
			printf("cannot accept connection ");
			exit(1);
		}
    
		// Show the IP address of the client.
		printf("connected to %s:%d",inet_ntoa(clientAddress.sin_addr),ntohs(clientAddress.sin_port));
		//cout << "  connected to " << inet_ntoa(clientAddress.sin_addr);

		NMove++;
		
		while (recv(connectSocket, &gameBoard, sizeof(game_board_struct), 0) > 0) {
		
			for(int i=1;i<5;++i){
				for(int j=1;j<9;++j){
					Board[i*9+j]=gameBoard.board[(i-1)*8+(j-1)];
				}
			}
			printf("\nturn :%d\n",(TPlayer)gameBoard.player);
			Turn=(TPlayer)gameBoard.player;
			
			ShowBoard(Board);
			printf("incoming: ");
			for(int i=0;i<32;i++)
				printf("%d ",gameBoard.board[i]);
			printf("\n");


			printf("iternal: ");
			for(int i=10;i<45;i++)
				printf("%d ",Board[i]);
			printf("\n");
			result.status=AI_NORMAL_MOVE;
			if(TMoveGenerator.NoMovesLeft(Board,Turn)){
				result.status=AI_HUMAN_WON;
				printf("You won\n");
			}else{
				TSearcher.SetSearchDepth(SarchDepth[Turn]);
				TSearcher.GetComputerMove(Board,Turn,Move);
				TMoveGenerator.MakeMove(Move,Board);
				ShowBoard(Board);
				if(TMoveGenerator.NoMovesLeft(Board,Other(Turn))){
					result.status=AI_COMPUTER_WON;
					printf("I won\n");
				}
			
				int ii;
				int jj;
				int newmove;
				for(int i=0;i<MAX_MOVE_LENGTH;++i){
					if(Move[i]==0){
						result.move[i]=-1;
						break;
					}
					
					ii=Move[i]/9;
					jj=Move[i]%9;
					newmove=((ii-1)*8+(jj-1));
					result.move[i]=newmove;

				}
				for(int i=0;i<25;i++){
					if(result.move[i]==-1)
						break;
					printf("%d %d\n",Move[i],result.move[i]);
				}
				printf("\n");
			}
			//sprintf(result.move,"%s",TSearcher.GetMoveInfoSend());
			//printf("\n%s\n",TSearcher.GetMoveInfoSend());
		
			// Send converted line back to client.
			if (send(connectSocket, &result, sizeof(result_struct), 0) < 0)
				printf("Error: cannot send modified data");
		}
		
	}
	printf("%s wins",PlayerName[Turn]);
}

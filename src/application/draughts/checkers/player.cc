enum TPlayer { WHITE=0 , BLACK=1 };

inline bool IsPiece( int Value ){
	return Value<4;
}

inline bool BelongsTo( int Piece, TPlayer Player){
	return Piece%2==Player;
}

inline TPlayer Other( TPlayer Player ){
	return (TPlayer)(1-Player);
}

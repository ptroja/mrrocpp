#ifndef CUBE_STATE_H_
#define CUBE_STATE_H_

enum CUBE_COLOR {UNKNOWN_CUBE_COLOR, RED, YELLOW, GREEN, BLUE, ORANGE, WHITE};

CUBE_COLOR read_cube_color (char input_char);

class CubeState
{
protected:
	// okresla jak sciany kostki sa zorientowane wzgledem chwytaka truck'a z punkltu widzenia chwytaka
	CUBE_COLOR up, down, front, rear, left, right;
	
public:
	char cube_tab[6][9]; // NAZWA DO ZMIANY

	friend class mp_task_rubik_cube_solver;
    // konstruktory
    CubeState();
    CubeState(CUBE_COLOR up_is, CUBE_COLOR down_is, CUBE_COLOR front_is, 
		CUBE_COLOR rear_is, CUBE_COLOR left_is, CUBE_COLOR right_is);
		
	// kontruktor kopiujacy
	CubeState (const CubeState& cs);
	
	// operatory
	CubeState& operator= (const CubeState& cs);
	
	// metody
	// okresla jak sciany kostki sa zorientowane wzgledem chwytaka truck'a z punkltu widzenia chwytaka
	void set_state(CUBE_COLOR up_is, CUBE_COLOR down_is, CUBE_COLOR front_is, 
		CUBE_COLOR rear_is, CUBE_COLOR left_is, CUBE_COLOR right_is);
		
	void print_face_color(CUBE_COLOR face_name);
	void print_cube_colors();
		
};

#endif /*CUBE_STATE_H_*/

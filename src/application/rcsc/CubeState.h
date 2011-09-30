#ifndef CUBE_STATE_H_
#define CUBE_STATE_H_

namespace mrrocpp {
namespace mp {
namespace common {

enum CUBE_COLOR {UNKNOWN_CUBE_COLOR, RED, YELLOW, GREEN, BLUE, ORANGE, WHITE};

CUBE_COLOR read_cube_color (char input_char);

class CubeState
{
protected:


public:
	// okresla jak sciany kostki sa zorientowane wzgledem chwytaka truck'a z punkltu widzenia chwytaka
	CUBE_COLOR up, down, front, rear, left, right;

	char cube_tab[6][9]; // TODO: NAZWA DO ZMIANY

	friend class rubik_cube_solver;
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
	void set_state(const CubeState &cubeState, int turnAngle);

	void print_face_color(CUBE_COLOR face_name);
	void print_cube_colors();
	CUBE_COLOR getUp() const;
	CUBE_COLOR getDown() const;
	CUBE_COLOR getFront() const;
	CUBE_COLOR getRear() const;
	CUBE_COLOR getLeft() const;
	CUBE_COLOR getRight() const;

};

} // namespace common
} // namespace mp
} // namespace mrrocpp

#endif /*CUBE_STATE_H_*/

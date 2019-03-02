#pragma once

class obstacles {
public:
	float seperation_distance = 3.0;
	float spring_constance = 30; //1/r2
	float position[3];
	float scale;

	void set_init(float x, float y, float z,float s) {
		this->position[0] = x;
		this->position[1] = y;
		this->position[2] = z;
		this->scale = s;
	}


	
};

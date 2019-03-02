#pragma once
struct quatern {
	float angle;
	float x;
	float y;
	float z;
};

#define PI 3.1415926
//convert angle to arc
float angle_arc(float angle) {
	float arc = angle / 180 * PI;
	return arc;
}


class sphere
{
public:

	float radius;
	float mass;
	float F[3];
	//linear 
	float velocity[3];
	float acceleration[3];
	//rotation
	quatern Q;
	float RMatrix[16];
	quatern QA;

	float position[3];
	bool collision;

	void setRadius(float r) {
		this->radius = r;
	}
	void setMass(float m) {
		this->mass = m;
	}
	void setInitialF(float f[3]) {
		this->F[0] = f[0];
		this->F[1] = f[1];
		this->F[2] = f[2];
	}
	void setInitialP(float p[3]) {
		this->position[0] = p[0];
		this->position[1] = p[1];
		this->position[2] = p[2];
	}

	/*	void unit(float x, float y, float z) {
	float length = pow(pow(Q[1], 2) + pow(Q[2], 2) + pow(Q[3], 2), 1.0f / 2.0f);
	Q[1] = Q[1] / length;
	Q[2] = Q[2] / length;
	Q[3] = Q[3] / length;
	}*/
	void cal_Matrix(quatern q) {
		int i;
		for (i = 0; i < 16; i++) {
			RMatrix[i] = 0;
		}
		float w = cos(angle_arc(q.angle) / 2);
		this->RMatrix[0] = 1 - 2 * pow(q.y, 2) - 2 * pow(q.z, 2);
		this->RMatrix[1] = 2 * q.x*q.y + 2 * w*q.z;
		this->RMatrix[2] = 2 * q.x*q.z - 2 * w*q.y;
		this->RMatrix[4] = 2 * q.x*q.y - 2 * w*q.z;
		this->RMatrix[5] = 1 - 2 * pow(q.x, 2) - 2 * pow(q.z, 2);
		this->RMatrix[6] = 2 * q.y*q.z + 2 * w*q.x;
		this->RMatrix[8] = 2 * q.x*q.z + 2 * w*q.y;
		this->RMatrix[9] = 2 * q.y*q.z - 2 * w*q.x;
		this->RMatrix[10] = 1 - 2 * pow(q.x, 2) - 2 * pow(q.y, 2);
		this->RMatrix[15] = 1;
	}

	quatern cal_quat(quatern q1, quatern q2) {
		quatern q;
		q.angle = q1.angle*q2.angle - (q1.x*q2.x + q1.y*q2.y + q1.z*q2.z);
		q.x = q1.angle*q2.x + q2.angle*q1.x + q1.y*q2.z - q1.z*q2.y;
		q.y = q1.angle*q2.y + q2.angle*q1.y + q1.z*q2.x - q1.x*q2.z;
		q.z = q1.angle*q2.z + q2.angle*q1.z + q1.x*q2.y - q1.y*q2.x;
		return q;
	}
};
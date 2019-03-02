#pragma once
#include<math.h>
#include "object.h"
#include"obstacles.h"
#include"math.h"

class flock:public sphere {
public:
	float seperation_distance=2.0; 
	float spring_constance=30; //1/r2
	float perception = 40.0;
	float collision_F[3]; //force to avoid collision
	float matching_velocity[3]; 
	float local_center[3]; //center of nearby boids
	float center[3] = { 0.0,0.0,-50.0 }; //control point to guide trajectory
	float center_sepe_distance = 10.0;
	float center_F[3]; //force applied by center point
	float collision_weight = 1.0;
	float center_weight = 0.1;
	float matching_weight = 0.1;
	int local_count; //number of	perceived boids
	float local_weight = 0.1;
	float spiral_F[3] = {0.0,0.0,0.0}; // force to spiral
	float spiral_weight = 3.0;

	void spiral() {
		float y = this->position[0] - this->center[0];// perpendicular (x,y)  (-y,x)
		float x = -(this->position[1] - this->center[1]);
		float length = pow(pow(x, 2) + pow(y, 2), 0.5);
		spiral_F[0] = x / length;
		spiral_F[1] = y / length;
		
	}

	void setcenter(float x,float y, float z) {
		this->center[0] = x;
		this->center[1] = y;
		this->center[2] = z;
	}

	void collision_avoidance(sphere &r) {
		float distance = pow(pow(this->position[0] - r.position[0], 2) + pow(this->position[1] - r.position[1], 2) + pow(this->position[2] - r.position[2], 2), 0.5);
		if (distance <= this->perception) {
			float direction[3];
			direction[0] = (this->position[0] - r.position[0]) / distance;
			direction[1] = (this->position[1] - r.position[1]) / distance;
			direction[2] = (this->position[2] - r.position[2]) / distance;
			float F = spring_constance / pow((distance - seperation_distance), 2);
			this->collision_F[0] = this->collision_F[0] + F*direction[0];
			this->collision_F[1] = this->collision_F[1] + F*direction[1];
			this->collision_F[2] = this->collision_F[2] + F*direction[2];
		}
	}

	void collision_obstacles(obstacles o) {
		float distance = pow(pow(this->position[0] - o.position[0], 2) + pow(this->position[1] - o.position[1], 2) , 0.5);
		if (distance <= this->perception) {
			float direction[3];
			direction[0] = (this->position[0] - o.position[0]) / distance;
			direction[1] = (this->position[1] - o.position[1]) / distance;
			//		direction[2] = (this->position[2] - o.position[2]) / distance;
			float F = spring_constance / pow((distance - seperation_distance), 2);
			this->collision_F[0] = this->collision_F[0] + F*direction[0];
			this->collision_F[1] = this->collision_F[1] + F*direction[1];
			//		this->collision_F[2] = this->collision_F[2] + F*direction[2];
		}

	}

	void centering() {
		float distance = pow(pow(this->position[0] - this->center[0], 2) + pow(this->position[1] - this->center[1], 2) + pow(this->position[2] - this->center[2], 2), 0.5);
			float direction[3];
			direction[0] = -(this->position[0] - this->center[0]) / distance;
			direction[1] = -(this->position[1] - this->center[1]) / distance;
			direction[2] = -(this->position[2] - this->center[2]) / distance;
			float F = spring_constance*(distance - center_sepe_distance)*fabs((distance - center_sepe_distance));
			this->center_F[0] = F*direction[0];
			this->center_F[1] = F*direction[1];
			this->center_F[2] = F*direction[2];
	
	}

	void cal_perception(sphere r) {
		float distance = pow(pow(this->position[0] - r.position[0], 2) + pow(this->position[1] - r.position[1], 2) + pow(this->position[2] - r.position[2], 2), 0.5);
		if (distance <= this->perception) {
			this->local_count++;
		}
	}




	void velocity_matching(sphere &r) {
		float distance = pow(pow(this->position[0] - r.position[0], 2) + pow(this->position[1] - r.position[1], 2) + pow(this->position[2] - r.position[2], 2), 0.5);
		if (distance <= this->perception) {
			this->matching_velocity[0] += r.velocity[0];
			this->matching_velocity[1] += r.velocity[1];
			this->matching_velocity[2] += r.velocity[2];
		}

	}

	void local_centering(sphere r) {
		float distance = pow(pow(this->position[0] - r.position[0], 2) + pow(this->position[1] - r.position[1], 2) + pow(this->position[2] - r.position[2], 2), 0.5);
		if (distance <= this->perception) {
			this->local_center[0] += r.position[0];
			this->local_center[1] += r.position[1];
			this->local_center[2] += r.position[2];
		}

	}

	void updateState(int count) {
		int i;
		for (i = 0; i < 3; i++) {

			this->matching_velocity[i] = this->matching_velocity[i] / float(count);
			this->local_center[i] = this->local_center[i] / float(count);
		}
		float collision_A[3];
		float center_A[3];
		float matching_A[3];
		float local_A[3];
		float spiral_A[3];
		//after multiply theri weights, add all the force and acceleraton together 
		for (i = 0; i < 3; i++) {
			collision_A[i]= this->collision_F[i] * this->collision_weight / this->mass;
			center_A[i]= this->center_F[i] * this->center_weight/this->mass;
			local_A[i] = (this->local_center[i] - this->position[i])*this->local_weight;
			matching_A[i] = (this->matching_velocity[i] - this->velocity[i])*this->matching_weight;
			spiral_A[i] = this->spiral_F[i]*this->spiral_weight / this->mass;
			this->acceleration[i] = collision_A[i] + center_A[i]+matching_A[i]+ local_A[i] +spiral_A[i]+ this->F[i] / this->mass;
			this->velocity[i] += this->acceleration[i];
			this->position[i] += this->velocity[i] * 1.0 / 500.0;
		}

		for (i = 0; i < 3; i++) {

			this->collision_F[i] = 0.0;
			this->matching_velocity[i] = 0.0;
			this->local_center[i] = 0.0;
			this->F[i] = 0.0;
		}
	}
};

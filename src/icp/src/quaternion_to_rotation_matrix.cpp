// This file simply transforms Quaternion vectors to Rotational Matrices.

#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>

int main (void) {

	// Quertonian transforms

	/**< Declaration of quaternion Position 1*/
	tf::Quaternion q;
	q.setX(-0.386);
	q.setY(0.741);
	q.setZ(0.463);
	q.setW(-0.297);


	/**< quaternion -> rotation Matrix*/
	tf::Matrix3x3 m1;
	m1.setRotation(q); 	

	for(int r = 0; r < 3; r++){
		for(int c = 0; c < 3; c++){
			std::cout << m1[r][c] << " ";
		}
	std::cout<<std::endl;
	}

	/**< Declaration of quaternion Position 2*/
	tf::Quaternion q2;
	q2.setX(-0.321);
	q2.setY(0.744);
	q2.setZ(0.474);
	q2.setW(-0.346);

	/**< quaternion -> rotation Matrix */
	tf::Matrix3x3 m2;
	m2.setRotation(q2); 	

	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			std::cout << m2[i][j] << " ";
		}
	std::cout<<std::endl;
	}


    /**< Declaration of quaternion Calib */
	tf::Quaternion q3;
	q3.setX(-0.002);
	q3.setY(-0.003);
	q3.setZ(1.000);
	q3.setW(-0.000);

	/**< quaternion -> rotation Matrix */
	tf::Matrix3x3 m3;
	m3.setRotation(q3); 	

	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			std::cout << m3[i][j] << " ";
		}
	std::cout<<std::endl;
	}

    return 0;
}
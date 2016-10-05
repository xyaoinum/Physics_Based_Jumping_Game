#include "RigidBodyLCPCollisionResolver.h"

void RigidBodyLCPCollisionResolver::resolveCollisions( std::vector<RigidBody>& rbs, const std::set<RigidBodyCollision>& rbcs )
{
  // Your code goes here!
	int R = rbs.size();
	int C = rbcs.size();
	if (C == 0)
		return;
	
	MatrixXs N(R * 3, C);
	MatrixXs Nfix(R * 3, C);
	std::vector<VectorXs> eta(C, VectorXs::Zero(R * 3));
	
	int i = 0;
	for (std::set<RigidBodyCollision>::iterator c = rbcs.begin(); c != rbcs.end(); c++, i++)
	{
		MatrixXs gamma = MatrixXs::Zero(2, R * 3);
		scalar theta0 = rbs[c->i0].getTheta();
		scalar theta1 = rbs[c->i1].getTheta();

    gamma.block(0, c->i0 * 3, 2, 2) = MatrixXs::Identity(2, 2);
    if (rbs[c->i0].getI() > 0)  // for zero moment of inertia, rotation is disabled
      gamma.block(0, c->i0 * 3 + 2, 2, 1) = Vector2s(-sin(theta0) * c->r0.x() - cos(theta0) * c->r0.y(), cos(theta0) * c->r0.x() - sin(theta0) * c->r0.y());
		
    gamma.block(0, c->i1 * 3, 2, 2) = -MatrixXs::Identity(2, 2);
    if (rbs[c->i1].getI() > 0)  // for zero moment of inertia, rotation is disabled
      gamma.block(0, c->i1 * 3 + 2, 2, 1) = Vector2s(sin(theta1) * c->r1.x() + cos(theta1) * c->r1.y(), -cos(theta1) * c->r1.x() + sin(theta1) * c->r1.y());
	
		eta[i] = -gamma.transpose() * c->nhat;
	}
	
	for (i = 0; i < C; i++)
	{
		N.block(0, i, R * 3, 1) = eta[i];
	}
	Nfix = N;
	for (int i = 0; i < R; i++)
	{
		if (rbs[i].isFixed())
		{
			Nfix.block(i * 3, 0, 3, C) = MatrixXs::Zero(3, C);
		}
	}
	
	VectorXs Minv(R * 3);
	for (int i = 0; i < R; i++)
	{
		Minv(i * 3 + 0) = Minv(i * 3 + 1) = 1 / rbs[i].getM();
		Minv(i * 3 + 2) = 1 / (rbs[i].getI() > 0 ? rbs[i].getI() : 1); // for zero moment of inertia the lack of rotation dof will be reflected in constraints, not here
	}

	MatrixXs A = N.transpose() * Minv.asDiagonal() * Nfix;
	
	VectorXs qdotminus(R * 3);
	for (int i = 0; i < R; i++)
	{
		qdotminus(i * 3 + 0) = rbs[i].getV().x();
		qdotminus(i * 3 + 1) = rbs[i].getV().y();
		qdotminus(i * 3 + 2) = rbs[i].getOmega();
	}
	VectorXs b = N.transpose() * qdotminus;
	
	VectorXs lambda(C);
	lcputils::solveLCPwithODE( A, b, lambda );
	
	VectorXs qdotplus = qdotminus + Minv.asDiagonal() * Nfix * lambda;		
	for (int i = 0; i < R; i++)
	{
		rbs[i].getV() = qdotplus.segment<2>(i * 3);
		rbs[i].getOmega() = qdotplus(i * 3 + 2);
	}
	
//	std::cout << "q.-: " << qdotminus.transpose() << std::endl;
//	std::cout << "dq: " << (qdotplus - qdotminus).transpose() << std::endl;
//	std::cout << "N: " << N << std::endl;
//	std::cout << "Nfix: " << Nfix << std::endl;
//	std::cout << "M: " << Minv << std::endl;
//	std::cout << "A: " << A << std::endl; 
//	std::cout << "b: " << b.transpose() << std::endl;
//	std::cout << "l: " << lambda.transpose() << std::endl;
	
	/*
  // Example of using the lcp solver
  MatrixXs A(4,4);
  A << 0.828869, 0.337798, -0.28125, -0.21875,
       0.337798, 0.828869, -0.21875, -0.28125,
       -0.28125, -0.21875, 0.828869, 0.337798,
       -0.21875, -0.28125, 0.337798, 0.828869;

  VectorXs b(4);
  b << -1, -1, -1, -1;

  VectorXs lambda(4);
  lcputils::solveLCPwithODE( A, b, lambda );
	 */
}

std::string RigidBodyLCPCollisionResolver::getName() const
{
  return "lcp";
}


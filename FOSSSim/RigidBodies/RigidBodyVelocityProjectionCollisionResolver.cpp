#include "RigidBodyVelocityProjectionCollisionResolver.h"


void RigidBodyVelocityProjectionCollisionResolver::resolveCollisions( std::vector<RigidBody>& rbs, const std::set<RigidBodyCollision>& rbcs )
{
  // Your code goes here!
	int R = rbs.size();
	int C = rbcs.size();
	if (C == 0)
		return;
	
	int V = 0;
	std::vector<int> vmap(R);
	for (int i = 0; i < R; i++)
	{
		if (!rbs[i].isFixed())
		{
			vmap[i] = V;
			V++;
		} else 
		{
			vmap[i] = -1;			
		}
	}
	
	MatrixXs N(V * 3, C);
	std::vector<VectorXs> eta(C, VectorXs::Zero(V * 3));
	
	int i = 0;
	for (std::set<RigidBodyCollision>::iterator c = rbcs.begin(); c != rbcs.end(); c++, i++)
	{
		MatrixXs gamma = MatrixXs::Zero(2, V * 3);
		scalar theta0 = rbs[c->i0].getTheta();
		scalar theta1 = rbs[c->i1].getTheta();
		
		if (!rbs[c->i0].isFixed())
		{
			gamma.block(0, vmap[c->i0] * 3, 2, 2) = MatrixXs::Identity(2, 2);
      if (rbs[c->i0].getI() > 0)  // for zero moment of inertia, rotation is disabled
        gamma.block(0, vmap[c->i0] * 3 + 2, 2, 1) = Vector2s(-sin(theta0) * c->r0.x() - cos(theta0) * c->r0.y(), cos(theta0) * c->r0.x() - sin(theta0) * c->r0.y());
		}
		
		if (!rbs[c->i1].isFixed())
		{
			gamma.block(0, vmap[c->i1] * 3, 2, 2) = -MatrixXs::Identity(2, 2);
      if (rbs[c->i1].getI() > 0)  // for zero moment of inertia, rotation is disabled
        gamma.block(0, vmap[c->i1] * 3 + 2, 2, 1) = Vector2s(sin(theta1) * c->r1.x() + cos(theta1) * c->r1.y(), -cos(theta1) * c->r1.x() + sin(theta1) * c->r1.y());
		}
		
		eta[i] = -gamma.transpose() * c->nhat;
	}
	
	for (i = 0; i < C; i++)
	{
		N.block(0, i, V * 3, 1) = eta[i];
	}
	/*
	Nfix = N;
	for (int i = 0; i < R; i++)
	{
		if (rbs[i].isFixed())
		{
			Nfix.block(i * 3, 0, 3, C) = MatrixXs::Zero(3, C);
		}
	}
	*/
	
	VectorXs M(V * 3);
	for (int i = 0; i < R; i++)
	{
		if (!rbs[i].isFixed())
		{
			M(vmap[i] * 3 + 0) = M(vmap[i] * 3 + 1) = rbs[i].getM();
			M(vmap[i] * 3 + 2) = (rbs[i].getI() > 0 ? rbs[i].getI() : 1); // for zero moment of inertia the lack of rotation dof will be reflected in constraints, not here
		}
	}
		
	VectorXs qdotminus(V * 3);
	for (int i = 0; i < R; i++)
	{
		if (!rbs[i].isFixed())
		{
			qdotminus(vmap[i] * 3 + 0) = rbs[i].getV().x();
			qdotminus(vmap[i] * 3 + 1) = rbs[i].getV().y();
			qdotminus(vmap[i] * 3 + 2) = rbs[i].getOmega();
		}
	}
	VectorXs mq = M.asDiagonal() * qdotminus;
	
//	std::cout << "M: " << M.transpose() << std::endl;
//	std::cout << "mq: " << mq.transpose() << std::endl;
//	std::cout << "N: " << N << std::endl;
	
	// Matrix in quadratic form of objective
	QuadProgPP::Matrix<scalar> G;
	// Equality constraints
	QuadProgPP::Matrix<scalar> CE;
	// Inequality constraints
	QuadProgPP::Matrix<scalar> CI;
	
	QuadProgPP::Vector<scalar> g0;
	QuadProgPP::Vector<scalar> ce0;
	QuadProgPP::Vector<scalar> ci0;
	QuadProgPP::Vector<scalar> x;
	
	G.resize(V * 3, V * 3);
	for (int i = 0; i < V * 3; i++) for (int j = 0; j < V * 3; j++) G[i][j] = (i == j ? M(i) : 0);
	g0.resize(V * 3);
	for (int i = 0; i < V * 3; i++) g0[i] = -mq(i);
	
	CE.resize(V * 3, 0);
	ce0.resize(0);
	
	CI.resize(V * 3, C);
	for (int i = 0; i < V * 3; i++) for (int j = 0; j < C; j++) CI[i][j] = N(i, j);
	ci0.resize(C);
	for (int i = 0; i < C; i++) ci0[i] = 0;
	
	/*
	std::cout << "G: " << G << std::endl;
	std::cout << "g0: " << g0 << std::endl;
	std::cout << "CI: " << CI << std::endl;
	std::cout << "ci0: " << ci0 << std::endl;
	 */

	x.resize(V * 3);
	solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
	
	for (int i = 0; i < R; i++)
	{
		if (!rbs[i].isFixed())
		{
			rbs[i].getV() = Vector2s(x[vmap[i] * 3 + 0], x[vmap[i] * 3 + 1]);
			rbs[i].getOmega() = x[vmap[i] * 3 + 2];
		}
	}
	
	//std::cout << "x: " << x << std::endl;

/*
  // Example of using QuadProg++
  // For detailed documentation, please see FOSSSim/quadprog/QuadProg++.hh
 
  // Matrix in quadratic form of objective
  QuadProgPP::Matrix<scalar> G;
  // Equality constraints
  QuadProgPP::Matrix<scalar> CE;
  // Inequality constraints
  QuadProgPP::Matrix<scalar> CI;
  
  QuadProgPP::Vector<scalar> g0;
  QuadProgPP::Vector<scalar> ce0;
  QuadProgPP::Vector<scalar> ci0;
  QuadProgPP::Vector<scalar> x;

  // M = 16  0 0
  //      0 16 0
  //      0  0 289.75
  G.resize(3,3);
  for(int i=0; i< 3; i++) for(int j=0; j< 3; j++) G[i][j]=0;
  G[0][0] = 16; 
  G[1][1] = 16;
  G[2][2] = 289.75;

  // -M \dot q = -0
  //             139.52
  //            -0
  g0.resize(3);
  g0[0] = 0;
  g0[1] = 139.52;
  g0[2] = 0;

  // No equality constraints, currently
  CE.resize(3,0);
  ce0.resize(0);

  // Compute the number of inequality constraints
  CI.resize(3,24);

  MatrixXs tempN(3,23);
  tempN << 0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
           1,  1,  1,  1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          -5, -4, -3, -2, 2, 3, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  for( int i = 0; i < 3; ++i ) for( int j = 0; j < 24; ++j ) CI[i][j] = tempN(i,j);

  // Constant term added to inequality constraints
  ci0.resize(24);
  for(int i=0; i< 24; i++) ci0[i] = 0;

  // Solution
  x.resize(3);

  solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
 */
}

std::string RigidBodyVelocityProjectionCollisionResolver::getName() const
{
  return "velocity-projection";
}


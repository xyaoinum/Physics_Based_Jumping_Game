#ifndef __RIGID_BODY_H__
#define __RIGID_BODY_H__

#include <Eigen/Core>
#include "FOSSSim/MathDefs.h"
#include "FOSSSim/MathUtilities.h"

#include <iostream>
#include <fstream>
#include <ctime>

class RigidBody
{
public:
    scalar timer = 1.0;
    int is_fragile;
    bool has_touched;
    
    const bool get_collision(){
        return in_collision;
    }
    
    void set_collision(bool b){
        in_collision = b;
    }
    
    const bool get_collision_enable(){
        return collision_enable;
    }
    
    void set_collision_enable(bool b){
        collision_enable = b;
    }
    
    const int get_max_x(){
        return max_x;
    }
    
    void set_max_x(int x){
        max_x = x;
    }
    
    const int get_min_x(){
        return min_x;
    }
    
    void set_min_x(int x){
        min_x = x;
    }
    
    const int get_dir(){
        return dir;
    }
    
    void set_dir(int x){
        dir = x;
    }
    
    void move_one_step(){
        int x = max_x%5;
        if(dir == 1){
            m_X(0) += 0.002*x+0.008;
        }else if(dir == -1){
            m_X(0) -= 0.002*x+0.008;
        }

        //std::cout << std::endl;
        //m_X = computeCenterOfMass(m_vertices,m_masses);
        //std::cout << "mx" << m_X(0) << " " << m_X(1) << std::endl;
    }


  // Inputs:
  //   v:         Velocity of the center of mass.
  //   omega:     Angular velocity of the rigid body.
  //   vertices:  Flat vector containing the vertices of the rigid body. The hull is defined
  //              in clockwise order by the vertices. Indices 2*i and 2*i+1 define the ith vertex.
  //              vertices.size() == 2*N
  //   masses:    Flat vector containing all masses associated with vertices. masses.size() == N
  //   radius:    Inflated radius of the rigid body for rendering and collision detection. These
  //              rigid bodies are minkowski sums of the poly-line hulls and circles. 
  RigidBody( const Vector2s& v, const scalar& omega, const VectorXs& vertices, const VectorXs& masses, const scalar& radius, const bool& fixed = false );
  virtual ~RigidBody() { }
  
  // Update quantities computed from 'core' state (e.g. rotation matrix derived from theta)
  void updateDerivedQuantities();

  // Returns true if this rigid body is fixed (immobile)
  bool isFixed() const;
  
  // Returns the tag
  std::string & tag() { return m_tag; }
  const std::string & tag() const { return m_tag; }  
  
  // Returns the center of mass
  Vector2s& getX();
  const Vector2s& getX() const;
  
  

  // Returns the orientation (an angle in radians)
  scalar& getTheta();
  const scalar& getTheta() const;
  
  // Returns the velocity of the center of mass
  Vector2s& getV();
  const Vector2s& getV() const;
  
  // Returns the angular velocity
  scalar& getOmega();
  const scalar& getOmega() const;

  // Returns the total mass of the rigid body
  const scalar& getM() const;

  // Returns the moment of inertia of the rigid body
  const scalar& getI() const;

  // Returns the radius of the rigid body
  const scalar& getRadius() const;

  // Returns the number of vertices in the rigid body
  int getNumVertices() const;
  const VectorXs & getVertices() const { return m_vertices; }

  // Returns the number of edges that compose the boundary of this rigid body
  int getNumEdges() const;



  // Returns the position of vertex i in 'world space', taking into account the body's orientation and position
  // Inputs:
  //   i:       Index of a vertex. Valid range is 0...N-1 where N is the number of vertices. 
  // Outputs:
  //   Position of a vertex in 'world space'
  Vector2s getWorldSpaceVertex( int i ) const;
  
  // Rotates a 'body space' vector into 'world space', taking into account the body's orientation
  // Inputs:
  //   bodyvec: A vector in 'body space'   
  // Outputs:
  //   The input 'body space' vector rotated by the body's orientation, but not translated. 
  Vector2s rotateIntoWorldSpace( const Vector2s& bodyvec ) const;
  
  // Returns the position of 'body space' vector bodyvec in 'world space', taking into account the body's orientation and position
  // Inputs:
  //   bodyvec: A vector in 'body space'
  // Outputs:
  //   The input 'body space' vector transformed into 'world space' 
  Vector2s computeWorldSpacePosition( const Vector2s& bodyvec ) const;
  
  // Given a 'world space' vector, computes the velocity assuming that point is on the rigid body
  // Inputs:
  //   worldposition: A vector in 'world space'
  // Outputs:
  //   The velocity of the input point assuming it was attached to the rigid body 
  Vector2s computeWorldSpaceVelocity( const Vector2s& worldposition ) const;

  Vector2s computeWorldSpaceVelocityGivenPositionRelativeToCM( const Vector2s& posnreltocm ) const;
  
  // Computes the i'th edge of the rigid body in 'world space'
  // Inputs:
  //   i:       Index of an edge. Valid range is 0...N-1 where N is the number of vertices. 
  // Outputs:
  //   ith edge of the rigid body in 'world space'
  Vector2s computeWorldSpaceEdge( int i ) const;
	
	// Returns the position of 'world space' vector worldvec in 'body space', taking into account the body's orientation and position
	// Inputs:
	//   worldvec: A vector in 'world space'
	// Outputs:
	//   The input 'world space' vector transformed into 'body space' 
	Vector2s computeBodySpacePosition( const Vector2s& worldvec ) const;
	


  // Returns a reference to a Vector2s containing the total force acting on the rigid body
  Vector2s& getForce();

  // Returns a reference to a scalar containing the total torque acting on the rigid body
  scalar& getTorque();



  // Computes the total momentum of this rigid body
  Vector2s computeTotalMomentum() const;

  // Computes the contribution to angular momentum from the center of mass' velocity
  scalar computeCenterOfMassAngularMomentum() const;
    
  // Computes the contribution to angular momentum from the angular velocity computed about the center of mass
  scalar computeSpinAngularMomentum() const;
  
  // Computes the total angular momentum of this rigid body
  scalar computeTotalAngularMomentum() const;

  // Computes the contribution to kinetic energy from the center of mass' velocity
  scalar computeCenterOfMassKineticEnergy() const;
  
  // Computes the contribution to kinetic energy from the angular velocity computed about the center of mass
  scalar computeSpinKineticEnergy() const;
  
  // Computes the total kinetic energy of the rigid body
  scalar computeKineticEnergy() const;



  // Writes the state of this rigid body to the provided stream. Only writes state that should change
  //  (center of mass, orientation, center of mass' velocity, angular velocity)
  void serialize( std::ofstream& outputstream ) const;

  // Loads the state of this rigid body from the provided outputstream. Only loads state that should change
  void deserialize( std::ifstream& inputstream );

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
      
      int max_x;
  int min_x;
  int dir;//-1 left, 1 right, 0 no move
      
      bool collision_enable;
      bool in_collision;
  // Computes the total mass of the rigid body.
  // Inputs:
  //   masses: A flat vector containing the masses of each vertex of the rigid body.
  // Outputs:
  //   Total mass of the rigid body.
  scalar computeTotalMass( const VectorXs& masses ) const;

  // Computes the center of mass of the rigid body.
  // Inputs:
  //   vertices:  Flat vector containing the vertices of the rigid body. The hull is defined
  //              in clockwise order by the vertices. Indices 2*i and 2*i+1 define the ith vertex.
  //              vertices.size() == 2*N
  //   masses:    Flat vector containing all masses associated with vertices. masses.size() == N
  // Outputs:
  //   Center of mass of the rigid body. 
  Vector2s computeCenterOfMass( const VectorXs& vertices, const VectorXs& masses ) const;

  // Computes the moment of inertia of the rigid body
  // Inputs:
  //   vertices:  Flat vector containing the vertices of the rigid body. The hull is defined
  //              in clockwise order by the vertices. Indices 2*i and 2*i+1 define the ith vertex.
  //              vertices.size() == 2*N
  //   masses:    Flat vector containing all masses associated with vertices. masses.size() == N
  // Outputs:
  //   Moment of inertia of the rigid body.
  scalar computeMomentOfInertia( const VectorXs& vertices, const VectorXs& masses ) const;
    
  
  /////////////////////////////////////////////////////////////////////////////
  // Constant variables

  bool m_fixed;
  
  // Total mass of the rigid body
  scalar m_M;
  // Masses of each point composing the rigid body (not required, but useful for debugging)
  VectorXs m_masses;
  // Moment of inertia of the rigid body
  scalar m_I;

  // Array containing body-space coordinates of vertices that compose the rigid body.
  VectorXs m_vertices;
  // Thickness of the rigid body. Rigid body is Minkowski sum of circle of this radius and 'centerline' of rigid body.
  scalar m_r;
  
  // Tag for scripting purposes
  std::string m_tag;


  /////////////////////////////////////////////////////////////////////////////
  // State variables

  // Center of mass of the rigid body.
  Vector2s m_X;
  // Orientation of the rigid body. Simply an angle in 2d. 
  scalar m_theta;
  // Velocity of the center of mass
  Vector2s m_V;
  // Angular velocity of the rigid body
  scalar m_omega;

  
  /////////////////////////////////////////////////////////////////////////////
  // Derived variables (cached mainly for efficency)
  
  // Orientation encoded as a 2x2 rotation matrix
  Matrix2s m_R;

  /////////////////////////////////////////////////////////////////////////////
  // Total force and torque

  Vector2s m_F;
  scalar m_tau;
  
};

#endif

#include "DragDampingForce.h"

DragDampingForce::DragDampingForce( const scalar& b )
: Force()
, m_b(b)
{
  assert( m_b >= 0.0 );
}

DragDampingForce::~DragDampingForce()
{}
  
void DragDampingForce::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size()%2 == 0 );

  std::cerr << outputmod::startred << "WARNING IN DRAGDAMPINGFORCE: " << outputmod::endred << "No energy defined for DragDampingForce." << std::endl;
}

void DragDampingForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == gradE.size() );
  assert( x.size()%2 == 0 );
  
  for( int i = 0; i < x.size()/2; ++i ) gradE.segment<2>(2*i) += m_b*v.segment<2>(2*i);
}

void DragDampingForce::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == hessE.rows() );
  assert( x.size() == hessE.cols() );
  assert( x.size()%2 == 0 );
  // Nothing to do.
}

void DragDampingForce::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == hessE.rows() );
  assert( x.size() == hessE.cols() );
  assert( x.size()%2 == 0 );
  
  hessE.diagonal().array() += m_b;
}

Force* DragDampingForce::createNewCopy()
{
  return new DragDampingForce(*this);
}

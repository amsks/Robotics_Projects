#ifndef _YOUR_PLANNER_H_
#define _YOUR_PLANNER_H_

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "RrtConConBase.h"
#include <cmath>

using namespace ::rl::plan;

/**
*	The implementation of your planner.
*	modify any of the existing methods to improve planning performance.
*/
class YourPlanner : public RrtConConBase
{
public:
  YourPlanner();

  virtual ~YourPlanner();

  virtual ::std::string getName() const;

  bool solve();

protected:
  void choose(::rl::math::Vector& chosen, bool origin);

  RrtConConBase::Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  RrtConConBase::Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  RrtConConBase::Neighbor
  nearest (const Tree& tree, const ::rl::math::Vector& chosen);

    struct VertexBundle
    {
        ::std::size_t index;

        ::rl::plan::VectorPtr q;

        ::rl::math::Real tmp;

        ::rl::math::Real exhaustCount;
    };

private:
    int currSteps;
};

#endif // _YOUR_PLANNER_H_

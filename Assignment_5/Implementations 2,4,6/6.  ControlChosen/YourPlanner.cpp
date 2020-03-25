#include "YourPlanner.h"
#include <rl/plan/SimpleModel.h>
#include <map>
#define M_OPT 0.1
#define BASIC 1
#define NEARESTCHECK 0
#define CONTROLSWAP 1
#define CONTROLCHOSEN 1

bool collision=FALSE;

YourPlanner::YourPlanner() :
  RrtConConBase()
{
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "Your Planner - NeighborCost Rev";
}

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
  //your modifications here
  RrtConConBase::choose(chosen);

}


YourPlanner::Vertex
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{

    //Do first extend step

    ::rl::math::Real distance = nearest.second;
    ::rl::math::Real step = distance;

    bool reached = false;

    if (step <= this->delta)
    {
        reached = true;
    }
    else
    {
        step = this->delta;
    }

    ::rl::plan::VectorPtr last = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

    // move "last" along the line q<->chosen by distance "step / distance"
    this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);

    this->model->setPosition(*last);
    this->model->updateFrames();


    if (this->model->isColliding())
    {

#if CONTROLSWAP
        collision=TRUE;
#endif
        return NULL;
    }

    ::rl::math::Vector next(this->model->getDof());

    while (!reached)
    {
        //Do further extend step

        distance = this->model->distance(*last, chosen);
        step = distance;

        if (step <= this->delta)
        {
            reached = true;
        }
        else
        {
            step = this->delta;
        }

        // move "next" along the line last<->chosen by distance "step / distance"
        this->model->interpolate(*last, chosen, step / distance, next);

        this->model->setPosition(next);
        this->model->updateFrames();

        if (this->model->isColliding())
        {
#if CONTROLSWAP
            collision=TRUE;
#endif
            break;
        }

        *last = next;
    }

    // "last" now points to the vertex where the connect step collided with the environment.
    // Add it to the tree
    Vertex connected = this->addVertex(tree, last);
    this->addEdge(nearest.first, connected, tree);
    return connected;

}

YourPlanner::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = (::std::min)(distance, this->delta);

  ::rl::plan::VectorPtr next = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *next);

  this->model->setPosition(*next);
  this->model->updateFrames();

  if (!this->model->isColliding())
  {

    Vertex extended = this->addVertex(tree, next);
    this->addEdge(nearest.first, extended, tree);
    return extended;
  }

  return NULL;
}

YourPlanner::Neighbor
YourPlanner::nearest(const Tree& tree,  ::rl::math::Vector& chosen)
{
    //create an empty pair <Vertex, distance> to return
    Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());

    //Iterate through all vertices to find the nearest neighbour
    for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
    {
        ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q);

        if (d < p.second)
        {
            p.first = *i.first;
            p.second = d;
        }
    }


    // Compute the square root of distance
    p.second = this->model->inverseOfTransformedDistance(p.second);

    return p;
}



bool
YourPlanner::solve()
{

  this->time = ::std::chrono::steady_clock::now();
  // Define the roots of both trees
  this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector >(*this->start));
  this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared< ::rl::math::Vector >(*this->goal));

  ::rl::math::Real MaxDistance= this->model->transformedDistance(*this->start, *this->goal);
  Tree* a = &this->tree[1];
  Tree* b = &this->tree[0];
    ::rl::math::Real C_vertex, C_opt,C_max;

    ::rl::math::Vector chosen(this->model->getDof());

  double m;

  while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
  {
    //First grow tree a and then try to connect b.
    //then swap roles: first grow tree b and connect to a.
    bool first=FALSE;
    for (::std::size_t j = 0; j < 2; ++j)
    {
        Neighbor aNearest;
      //Sample a random configuration
#if BASIC

        this->choose(chosen);
#if CONTROLCHOSEN
        ::rl::math::Real distancechosen = this->model->transformedDistance(chosen,*this->goal);
        while(distancechosen>MaxDistance){
            this->choose(chosen);
            distancechosen = this->model->transformedDistance(chosen,*this->goal);
        }
#endif
        aNearest=this->nearest(*a, chosen);


#elif  NEARESTCHECK

        double r;
        double m_quality;


        if(first) {
            do {

                this->choose(chosen);
#if CONTROLCHOSEN
        ::rl::math::Real distancechosen = this->model->transformedDistance(chosen,*this->goal);
        while(distancechosen>MaxDistance){
            this->choose(chosen);
            distancechosen = this->model->transformedDistance(chosen,*this->goal);
        }
#endif
                aNearest = this->nearest(*a, chosen);


                for (VertexIteratorPair i = ::boost::vertices(*a); i.first != i.second; ++i.first) {
                    ::rl::math::Real d = this->model->transformedDistance(*(*a)[*i.first].q, *this->start)
                            + this->model->transformedDistance(*(*a)[*i.first].q, *this->goal);
                    if (d > C_max) {
                        C_max = d;
                    }
                }
                if (m_quality > M_OPT) m_quality = M_OPT;
                C_opt = this->model->transformedDistance(*this->goal, *this->start);
                C_vertex = this->model->transformedDistance(*(*a)[aNearest.first].q, *this->start) +
                           this->model->transformedDistance(*(*a)[aNearest.first].q, *this->goal);
                m_quality = 1.0 - (double) ((C_vertex - C_opt) / (C_max - C_opt));
                r = (double)((rand()%1000))/1000;

            } while (r >= m_quality);
        }else{
            this->choose(chosen);
#if CONTROLCHOSEN
            ::rl::math::Real distancechosen = this->model->transformedDistance(chosen,*this->goal);
            while(distancechosen>MaxDistance){
                this->choose(chosen);
                distancechosen = this->model->transformedDistance(chosen,*this->goal);
            }
#endif
            aNearest = this->nearest(*a, chosen);
        }

        first=TRUE;
#endif
      //Do a CONNECT step from the nearest neighbour to the sample

      Vertex aConnected = this->connect(*a, aNearest, chosen);

      //If a new node was inserted tree a
      if (NULL != aConnected)
      {
        // Try a CONNECT step form the other tree to the sample
        Neighbor bNearest = this->nearest(*b, *(*a)[aConnected].q);
        Vertex bConnected = this->connect(*b, bNearest, *(*a)[aConnected].q);

        if (NULL != bConnected)
        {
          //Test if we could connect both trees with each other
          if (this->areEqual(*(*a)[aConnected].q, *(*b)[bConnected].q))
          {

            this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
            this->end[1] = &this->tree[1] == b ? bConnected : aConnected;
            return true;
          }
        }
      }

      //Swap the roles of a and b
      using ::std::swap;
#if CONTROLSWAP
      if(collision){
          swap(a, b);
          collision=FALSE;
      }
#else
        swap(a, b);
#endif


    }

  }

  return false;
}


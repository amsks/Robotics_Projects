#include "YourPlanner.h"
#include <rl/plan/SimpleModel.h>
#include <iostream>

#define EXHAUSTED 15
#define COLLTIMES 5
#define EXHAUST false
#define HEUR false
#define RANDOMPCT 100
#define STEPSIZE 5
#define STEPLIMIT false
#define CONNECTTREESPCT 0
#define SWAPSTEPS 5000

bool collision= FALSE ;

YourPlanner::YourPlanner() :
  RrtConConBase()
{
    currSteps = 0;
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "Your Planner - SwapLimiter with Collision";
}

//try to connect to a random sample of the other tree

void
YourPlanner::choose(::rl::math::Vector& chosen, bool origin)
{
    int pctRand = RANDOMPCT  ; //60
    int rand = std::rand()%100;
    if(rand<pctRand) {
        chosen = this->sampler->generate();
    }
    else if(origin){
        chosen = *this->goal;
    }
    else{
        chosen = *this->start;
    }
}

RrtConConBase::Vertex 
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
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
        tree[nearest.first].tmp += 1;
        collision = TRUE ; 
        return NULL;
    }

    ::rl::math::Vector next(this->model->getDof());
    int count = 0;
    while (!reached && (count < STEPSIZE || !STEPLIMIT) && currSteps < SWAPSTEPS)
    {
        currSteps ++;
        count++;
        if (count == 40){
            count = 40;
        }
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
            tree[nearest.first].tmp += 1;
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

RrtConConBase::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  return RrtConConBase::extend(tree, nearest, chosen);
}

bool
YourPlanner::solve()
{
    this->time = ::std::chrono::steady_clock::now();
    // Define the roots of both trees
    ::rl::math::Vector* _start = this->goal;
    this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector >(*this->start));
    this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared< ::rl::math::Vector >(*this->goal));

    Tree* a = &this->tree[0];
    Tree* b = &this->tree[1];

    ::rl::math::Vector chosen(this->model->getDof());


    while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
    {
        //First grow tree a and then try to connect b.
        //then swap roles: first grow tree b and connect to a.
        for (::std::size_t j = 0; j < 2; ++j)
        {
            //Sample a random configuration
            int random = std::rand()%100;
            if(random<CONNECTTREESPCT){
                Tree* c;
                if(j==0){
                    c = b;
                }
                else if(j==1){
                    c = a;
                }
                int count = 0;
                int randomVal = std::rand() % boost::num_vertices(*c);
                for (VertexIteratorPair i = ::boost::vertices(*c); i.first != i.second; ++i.first)
                {
                    count++;
                    if(count == randomVal){
                        chosen = *((*c)[*i.first].q);
                    }
                }
            }
            else {
                this->choose(chosen, j % 2 == 0);
            }
            this->choose(chosen, j % 2 == 0);
            //Find the nearest neighbour in the tree
            Neighbor aNearest = this->nearest(*a, chosen);

            //Do a CONNECT step from the nearest neighbour to the sample
            Vertex aConnected = this->connect(*a, aNearest, chosen);

            //If a new node was inserted tree a
            if (NULL != aConnected)
            {
                //collision = FALSE ;
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
            if( currSteps>=SWAPSTEPS ){
                currSteps = 0;
                using ::std::swap;
                swap(a, b);
                //collision = FALSE ;
            }
        }

    }

    return false;
}

RrtConConBase::Neighbor
YourPlanner::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{
    //create an empty pair <Vertex, distance> to return
    bool own = true;
    bool heur = false;
    if(HEUR || EXHAUST){
    Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());

    //Iterate through all vertices to find the nearest neighbour
    for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
    {
        ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q);

        ::rl::math::Real fails = tree[*i.first].tmp;
        if(HEUR) {
            double failval = std::abs(COLLTIMES - fails);
            double heurMod = 0.9 + (0.1 / COLLTIMES * failval);
            if (d * heurMod < p.second) {
                p.first = *i.first;
                p.second = d;
            } else if (fails >= EXHAUSTED) {
                int inte = fails;
            }
        }
        else{
            if (d < p.second && fails<EXHAUSTED) {
                p.first = *i.first;
                p.second = d;
            }
        }
    }


    // Compute the square root of distance
    p.second = this->model->inverseOfTransformedDistance(p.second);

    return p;
    }
    else{
        return RrtConConBase::nearest(tree, chosen);
    }
}


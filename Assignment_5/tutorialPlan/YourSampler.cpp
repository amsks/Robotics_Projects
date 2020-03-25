#include <chrono>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"
#include <iostream>

#define PASSAGEPCT 0
#define BORDERPCT 0
#define NEIGHBOURNUM 5
#define BORDERSTEP 0.02
#define GO while
#define CRAZY (true)

namespace rl
{
    namespace plan
    {
        YourSampler::YourSampler() :
            Sampler(),
            randDistribution(0, 1),
            randEngine(::std::random_device()())
        {
        }

        YourSampler::~YourSampler()
        {

        }

        ::rl::math::Vector
        YourSampler::generate()
        {
            return generateUniform();
            int rand = std::rand()%100;
            if(rand < PASSAGEPCT) {
                return generatePassage();
            }
            else if (rand < (PASSAGEPCT + BORDERPCT +  0)) {
                return generateBorder();
            }
            else{
                return generateUniform();
            }
        }

        ::rl::math::Vector
        YourSampler::generateUniform()
        {
            ::rl::math::Vector rand(this->model->getDof());

            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                rand(i) = this->rand();
            }
            return this->model->generatePositionUniform(rand);
        }

        ::rl::math::Vector YourSampler::generatePassage() {
             int collCount = 0;
            ::rl::math::Vector pos;
             while(collCount<1 || collCount>4) {
                 collCount = 0;
                 ::rl::math::Vector rand(this->model->getDof());
                 for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                 {
                     rand(i) = this->rand();
                 }
                 pos = this->model->generatePositionUniform(rand);
                 ::rl::math::Vector sigma(this->model->getDof());
                 for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                 {
                     sigma(i) = 0.1;
                 }
                 //std::cout << "Neighbours: " << std::endl;
                 for (int j = 0; j < NEIGHBOURNUM; j++) {
                     for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
                         rand(i) = this->rand();
                     }
                     ::rl::math::Vector neigh = this->model->generatePositionGaussian(rand, pos, sigma);
                     //std::cout << neigh << std::endl;
                     //std::cout << j << "th neighbour" << std::endl;
                     this->model->setPosition(neigh);
                     this->model->updateFrames();
                     if (this->model->isColliding()) {
                         collCount++;
                     }
                 }
             }
            return pos;
        }

        //::rl::math::Vector YourSampler::generateGaussian() {
        //    ::rl::math::Vector goal =
        //}

        ::rl::math::Vector YourSampler::generateBorder() {
           ::rl::math::Vector pos;
           GO CRAZY {
                ::rl::math::Vector rand(this->model->getDof());
                for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                {
                    rand(i) = this->rand();
                }
                pos = this->model->generatePositionUniform(rand);
                this->model->setPosition(pos);
                this->model->updateFrames();
                if (this->model->isColliding()) {
                    break;
                }
            }
            int crazyCount = 0;
            GO CRAZY{
                crazyCount++;
                bool found = false;
                ::rl::math::Vector borderSample;
                ::rl::math::Vector rand(this->model->getDof());
                ::rl::math::Vector sigma(this->model->getDof());
                for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
                    sigma(i) = crazyCount * BORDERSTEP;
                    rand(i) = this->rand();
                }
                for(int j = 0; j<5; j++) {
                    ::rl::math::Vector sample = this->model->generatePositionGaussian(rand, pos, sigma);
                    this->model->setPosition(sample);
                    this->model->updateFrames();
                    if (!this->model->isColliding()) {
                        return sample;
                    }
                }
                if(crazyCount>50){return generateUniform();}
           }
        }

        ::std::uniform_real_distribution< ::rl::math::Real>::result_type
        YourSampler::rand()
        {
            return this->randDistribution(this->randEngine);
        }

        void
        YourSampler::seed(const ::std::mt19937::result_type& value)
        {
            this->randEngine.seed(value);
        }

    }
}

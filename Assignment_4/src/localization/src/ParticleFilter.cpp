#include "localization/ParticleFilter.h"
#include "localization/Util.h" 

#include "tf/tf.h"

using namespace std;

ParticleFilter::ParticleFilter(int numberOfParticles) {
	this->numberOfParticles = numberOfParticles;

	// initialize particles
	for (int i = 0; i < numberOfParticles; i++) {
		this->particleSet.push_back(new Particle());
	}

	// this variable holds the estimated robot pose
	this->bestHypothesis = new Particle();

	// at each correction step of the filter only the laserSkip-th beam of a scan should be integrated
	this->laserSkip = 5;

	// distance map used for computing the likelihood field
	this->distMap = NULL;
}

ParticleFilter::~ParticleFilter() {
	// delete particles
	for (int i = 0; i < numberOfParticles; i++) {
		Particle* p = this->particleSet[i];
		delete p;
	}

	this->particleSet.clear();

	if (this->likelihoodField)
		delete[] this->likelihoodField;

	delete this->bestHypothesis;

	if (this->distMap)
		delete[] this->distMap;
}

int ParticleFilter::getNumberOfParticles() {
	return this->numberOfParticles;
}

std::vector<Particle*>* ParticleFilter::getParticleSet() {
	return &(this->particleSet);
}

void ParticleFilter::initParticlesUniform() {
	//get map properties
	int mapWidth, mapHeight;
	double mapResolution;
	this->getLikelihoodField(mapWidth, mapHeight,mapResolution);

	// @Aditya: Initial template of code. WIll have to test it out 
	double x_min = 0 ;
	double x_max = mapWidth ;
	double y_min = 0 ; 
	double y_max = mapHeight ; 
	double theta_min = 0 ;
	double theta_max = 2*M_PI ;   	

	double particle_x, particle_y, particle_theta ; 

	for( int i=0; i<this->numberOfParticles; i++){
		
		particle_x = Util::uniformRandom ( x_min, x_max ) ; 
		particle_y = Util::uniformRandom ( y_min, y_max ) ; 
		particle_theta = Util::uniformRandom ( theta_min, theta_max ) ; 

		Particle* P = new(Particle) ; 

		P-> x = particle_x * mapResolution;
		P-> y = particle_y * mapResolution;
		P-> theta = particle_theta ;
		P-> weight = 1.0 ; 								// All weights are initialized to the value of 1

		this->particleSet[i] = P ; 

		//delete ( P )									// @@Aditya: Check for memory leakage

	}
}

void ParticleFilter::initParticlesGaussian(double mean_x, double mean_y,
		double mean_theta, double std_xx, double std_yy, double std_tt) {

	// @Aditya: Initial template of code. WIll have to test it out

	

	double x_gaussian, y_gaussian, t_gaussian ; 

	for ( int i=0; i<this->numberOfParticles; i++ ){
		x_gaussian = Util::gaussianRandom ( mean_x, std_xx ) ; 
		y_gaussian = Util::gaussianRandom ( mean_y, std_yy ) ; 
		t_gaussian = Util::gaussianRandom ( mean_theta, std_tt ) ; 

		Particle* P = new Particle( x_gaussian, y_gaussian, t_gaussian, 1.0 ) ;					// All weights are initialized to the value of 1

		this->particleSet[i] = P ;  

	}

}

/**
 *  Initializes the likelihood field as our sensor model.
 */
void ParticleFilter::setMeasurementModelLikelihoodField(
		const nav_msgs::OccupancyGrid& map, double zRand, double sigmaHit) {
	ROS_INFO("Creating likelihood field for laser range finder...");

	// create the likelihood field - with the same discretization as the occupancy grid map
	this->likelihoodField = new double[map.info.height * map.info.width];
	this->likelihoodFieldWidth = map.info.width;
	this->likelihoodFieldHeight = map.info.height;
	this->likelihoodFieldResolution = map.info.resolution;

  // calculates the distance map and stores it in member variable 'distMap'
	// for every map position it contains the distance to the nearest occupied cell.
	calculateDistanceMap(map);

	// Here you have to create your likelihood field
	// HINT0: sigmaHit is given in meters. You have to take into account the resolution of the likelihood field to apply it.
	// HINT1: You will need the distance map computed 3 lines above
	// HINT2: You can visualize it in the map_view when clicking on "show likelihood field" and "publish all".
	// HINT3: Storing probabilities in each cell between 0.0 and 1.0 might lead to round-off errors, therefore it is
	// good practice to convert the probabilities into log-space, i.e. storing log(p(x,y)) in each cell. As a further
	// advantage you can simply add the log-values in your sensor model, when you weigh each particle according the
	// scan, instead of multiplying the probabilities, because: log(a*b) = log(a)+log(b).

	// TODO: here comes your code
	
	/* P(Z/x, m) = Z_hit * P_hit + z_rand * P_rand  
		Z_hit -> get it from distance map, and it is the set of obstacle positions
		P_hit -> A aussian centered around the distance to the closest obstale and with std_dev = sigmaHit
		Z_rand -> given as argument 
		P_rand -> Set to 1 
	*/ 

	//@Aditya : Check it in case code fails

	// for ( int i=0; i<this->likelihoodFieldHeight * this->likelihoodFieldWidth; i++){
		
	// 	double p_hit = Util::gaussian ( 0, sigmaHit/this->likelihoodFieldResolution, distMap[i]) ; 

	// 	this->likelihoodField[i] = log((1-zRand)* p_hit + zRand) ;
	// }	

	ROS_INFO("...DONE creating likelihood field!");
}

void ParticleFilter::calculateDistanceMap(const nav_msgs::OccupancyGrid& map) {
	// calculate distance map = distance to nearest occupied cell
	distMap = new double[likelihoodFieldWidth * likelihoodFieldHeight];
	int occupiedCellProbability = 90;
	// initialize with max distances
	for (int x = 0; x < likelihoodFieldWidth; x++) {
		for (int y = 0; y < likelihoodFieldHeight; y++) {
			distMap[x + y * likelihoodFieldWidth] = 32000.0;
		}
	}
	// set occupied cells next to unoccupied space to zero
	for (int x = 0; x < map.info.width; x++) {
		for (int y = 0; y < map.info.height; y++) {
			if (map.data[x + y * map.info.width] >= occupiedCellProbability) {
				bool border = false;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						if (!border && x + i >= 0 && y + j >= 0 && x + i
								< likelihoodFieldWidth && y + j
								< likelihoodFieldHeight && (i != 0 || j != 0)) {
							if (map.data[x + i + (y + j) * likelihoodFieldWidth]
									< occupiedCellProbability && map.data[x + i
									+ (y + j) * likelihoodFieldWidth] >= 0)
								border = true;
						}
						if (border)
							distMap[x + i + (y + j) * likelihoodFieldWidth]
									= 0.0;
					}
				}
			}
		}
	}
	// first pass -> SOUTHEAST
	for (int x = 0; x < likelihoodFieldWidth; x++)
		for (int y = 0; y < likelihoodFieldHeight; y++)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}

	// second pass -> NORTHWEST
	for (int x = likelihoodFieldWidth - 1; x >= 0; x--)
		for (int y = likelihoodFieldHeight - 1; y >= 0; y--)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}
}

double* ParticleFilter::getLikelihoodField(int& width, int& height,
		double& resolution) {
	width = this->likelihoodFieldWidth;
	height = this->likelihoodFieldHeight;
	resolution = this->likelihoodFieldResolution;

	return this->likelihoodField;
}

/**
 *  A generic measurement integration method that invokes some specific observation model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::measurementModel(
		const sensor_msgs::LaserScanConstPtr& laserScan) {
	likelihoodFieldRangeFinderModel(laserScan);
}

/**
 *  Method that implements the endpoint model for range finders.
 *  It uses a precomputed likelihood field to weigh the particles according to the scan and the map.
 */
void ParticleFilter::likelihoodFieldRangeFinderModel(
		const sensor_msgs::LaserScanConstPtr & laserScan) {
 		
	// for ( int i=0; i<this->numberOfParticles; i++){
	// 	double weight = 0 ; 

	// double sum = 0 ; 

	// //
	// 	for (int j=0; laserScan->angle_min + double(j)*laserScan->angle_increment < laserScan->angle_max; j += this->laserSkip ){
	// 		double range = laserScan->ranges[j];
	// 		// @Aditya: check for the validity of the range
	// 		//double range = j;
	// 		double current_angle = laserScan->angle_min + double(j)*laserScan->angle_increment + particleSet[i]->theta; 
	// 		if ( (range >= laserScan->range_min) && (range <= laserScan->range_max) ){
	// 			int hitX = (particleSet[i]->x + range*cos( current_angle ))  / (this->likelihoodFieldResolution); 

	// 			int hitY = (particleSet[i]->y + range*sin( current_angle )) / (this->likelihoodFieldResolution);
	// 			if (hitX >= 0 && hitX < this->likelihoodFieldWidth && hitY >= 0 && hitY < this->likelihoodFieldHeight){
	// 				weight += this->likelihoodField [ hitX + hitY * this->likelihoodFieldWidth ] ; 
	// 			}
	// 			else{
	// 				weight += log(1e-10);
	// 			}
	// 		}	 
	// 	}
	// 	this->particleSet[i]->weight = exp(weight) ;
	// 	sum += exp(weight) ; 
	// }


	// for(int i = 0; i < this->numberOfParticles; i++){
	// 	this->particleSet[i]->weight = this->particleSet[i]->weight / sum;
	// }

}


void ParticleFilter::setMotionModelOdometry(double alpha1, double alpha2,
		double alpha3, double alpha4) {
	this->odomAlpha1 = alpha1;
	this->odomAlpha2 = alpha2;
	this->odomAlpha3 = alpha3;
	this->odomAlpha4 = alpha4;

}

/**
 *  A generic motion integration method that invokes some specific motion model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::sampleMotionModel(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	sampleMotionModelOdometry(oldX, oldY, oldTheta, newX, newY, newTheta);
}

/**
 *  Method that implements the odometry-based motion model.
 */
void ParticleFilter::sampleMotionModelOdometry(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	// TODO: here comes your code

		double delta_x = newX - oldX ; 
	double delta_y = newY - oldY ; 

	double delta_trans = abs(sqrt( pow( delta_x,2.0) + pow(delta_y,2.0) ) ) ; 
	double delta_rot1 = Util::diffAngle(oldTheta,atan2(delta_y, delta_x)); 							
	double delta_rot2 = Util::diffAngle(delta_rot1, (newTheta - oldTheta)); 



	for ( int i=0; i<this->numberOfParticles; i++) {
		double error_trans = Util::gaussianRandom ( 0, this->odomAlpha3*delta_trans + this->odomAlpha4*(abs(Util::normalizeTheta(delta_rot1 + delta_rot2))) ) ; 
		double error_rot1  = Util::gaussianRandom ( 0, this->odomAlpha1*abs(delta_rot1) + this->odomAlpha2*delta_trans ) ; 
		double error_rot2  = Util::gaussianRandom ( 0, this->odomAlpha1*abs(delta_rot2) + this->odomAlpha2*delta_trans ) ;

		double hat_trans = delta_trans + error_trans ; 
		double hat_rot1  = delta_rot1  + error_rot1 ;
		double hat_rot2  = delta_rot2  + error_rot2 ;

		this->particleSet[i]->x += hat_trans*cos(Util::normalizeTheta(this->particleSet[i]->theta + hat_rot1)) ;
		this->particleSet[i]->y += hat_trans*sin(Util::normalizeTheta(this->particleSet[i]->theta + hat_rot1)) ; 
		this->particleSet[i]->theta = Util::normalizeTheta(this->particleSet[i]->theta + hat_rot1 + hat_rot2) ; 	
	}

}

/**
 *  The stochastic importance resampling.
 */
void ParticleFilter::resample() {
	//TODO: here comes your code
	// std::vector<Particle*> resamples(this->numberOfParticles);  
	
	// std::vector<double> weight_wheel(this->numberOfParticles);

	// this->sumOfParticleWeights = 0;
	// for(int i = 0; i < this->numberOfParticles; i++){
	// 	this->sumOfParticleWeights += this->particleSet[i]->weight;
	// }

	// for(int i = 0; i < this->numberOfParticles; i++){
	// 	this->particleSet[i]->weight = this->particleSet[i]->weight / this->sumOfParticleWeights;
	// }


	// weight_wheel[0] = this->particleSet[0]->weight;
	// for(int i = 1; i < this->numberOfParticles; i++){
	// 	weight_wheel[i] = weight_wheel[i-1] + this->particleSet[i]->weight;
	// }

	// double rand_angle = Util::uniformRandom(0,1/this->numberOfParticles);
	// double angle_increment = 1/this->numberOfParticles;

	// for(int i = 0, j= 0;i < this->numberOfParticles; i++ ){
	// 	while (rand_angle > weight_wheel[j])
	// 	{
	// 		j++;
	// 	}
	// 	Particle* P = new Particle( this->particleSet[j]) ;					// All weights are initialized to the value of 1

	// 	resamples[i] = P;

	// 	rand_angle += angle_increment;
	// }

	// delete old particles
	// for (int i = 0; i < numberOfParticles; i++) {
	// 	Particle* p = this->particleSet[i];
	// 	delete p;
	// }
	
	// this->particleSet = resamples ;  							// Finally assign resamples to ParticleSet
	

	// double max_index = 0;
	// for ( int i=1; i<this->numberOfParticles; i++){
	// 	if ( resamples[i]->weight > resamples[max_index]->weight )
	// 		max_index = i ; 
	// }

	// this->bestHypothesis = resamples[max_index] ; 

	// Implementation of the low variance sampling algo

	// int M = this->numberOfParticles ; 
	// std::vector<double> W ; 
	// std::vector<Particle*> resamples(this->numberOfParticles);
	// double sum_weights ; 

	// calculate the sum of the particle weights because meh
	// for (int x=0; x<M; x++){
	// 	sum_weights = sum_weights + this->particleSet[x]->weight ;
	// }

	// normalize the weights and store them in an array
	// for (int i=0; i<M; i++){
	// 	W.push_back(this->particleSet[i]->weight/sum_weights) ; 
	// }

	// double r = Util::uniformRandom(0,1/M) ; 			// Generate a random number between 0 and 1/M
	// double c = W[0] ; 									// Assign the first weight as the seding value for algorithm
	// int k = 1 ;  										// Used as i in the book

	// for(int m=0; m<M; m++){
	// 	int u = r + (m-1)*(1/M) ; 						// assign u as a random number
	// 	while ( u > c){
	// 		k = k + 1 ; 								// increment k and add weights to c till it  reahes u
	// 		c = c + W[k] ; 
	// 	}

	// 	resamples[k] = this->particleSet[k] ; 	// the value of k when the while loop breaks, is the particle that is assigned to the resampled vector	
	// } 

	// this->particleSet = resamples ;

	// double max_index = 0;
	// for ( int i=1; i<this->numberOfParticles; i++){
	// 	if ( resamples[i]->weight > resamples[max_index]->weight )
	// 		max_index = i ; 
	// }

	// this->bestHypothesis = resamples[max_index] ; 
}

Particle* ParticleFilter::getBestHypothesis() {
	return this->bestHypothesis;
}

// added for convenience
int ParticleFilter::computeMapIndex(int width, int height, int x,
		int y) {
	return x + y * width;
}


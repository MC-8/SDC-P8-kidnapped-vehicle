/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

#define EPS (1e-90)

//#define DEBUG_LANDMARK_LIST_IN_RANGE
//#define DEBUG_OBSERVATION_MAPPING
//#define DEBUG_DATA_ASSOCIATION
//#define DEBUG_DATA_ASSOCIATION_INNER_LOOP
// #define DEBUG_WEIGHT_CALC

#define DEBUG_PART_NUM (0)

void ParticleFilter::init(double x, double y, double theta, double std[])
{
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	std::default_random_engine generator;

	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];

	std::normal_distribution<double> distribution_x(x, std_x);
	std::normal_distribution<double> distribution_y(y, std_y);
	std::normal_distribution<double> distribution_theta(theta, std_theta);

	num_particles = 500;
	for (int i = 0; i < num_particles; ++i)
	{
		Particle particle;
		particle.x = distribution_x(generator);
		particle.y = distribution_y(generator);
		particle.theta = distribution_theta(generator);
		particle.weight = 1;
		particles.push_back(particle);
		weights.push_back(particle.weight); // Oddly enough, there is a separate vector for weights
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::default_random_engine generator;

	for (int i = 0; i < num_particles; ++i)
	{
		if (abs(yaw_rate) > EPS) // non-zero yaw rate
		{
			particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
		}
		else
		{
			particles[i].x += delta_t * velocity * cos(particles[i].theta);
			particles[i].y += delta_t * velocity * sin(particles[i].theta);
		}
		particles[i].theta += delta_t * yaw_rate;

		// Add random Gaussian noise
		std::normal_distribution<double> distribution_x(0, std_pos[0]);
		std::normal_distribution<double> distribution_y(0, std_pos[1]);

		particles[i].x += distribution_x(generator);
		particles[i].y += distribution_y(generator);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations)
{
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
	for (unsigned int i_obs = 0; i_obs < observations.size(); ++i_obs)
	{
		double distance = std::numeric_limits<double>::max();
		for (unsigned int i_pred = 0; i_pred < predicted.size(); ++i_pred)
		{
			double temp_dist = dist(predicted[i_pred].x, predicted[i_pred].y, observations[i_obs].x, observations[i_obs].y);
			if (temp_dist < distance)
			{
				distance = temp_dist;
				observations[i_obs].id = predicted[i_pred].id;
#ifdef DEBUG_DATA_ASSOCIATION_INNER_LOOP
				std::cout << "i_obs = " << i_obs << " \ti_pred = " << i_pred << " \t| observations[i_obs].id= " << observations[i_obs].id << std::endl;
#endif
			}
		}
		if (distance == std::numeric_limits<double>::max())
		{
			std::cout << "\nSomething may be wrong with ParticleFilter::dataAssociation. Could not find nearest point.";
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
								   const std::vector<LandmarkObs> &observations, const Map &map_landmarks)
{
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// Transform observations in MAP coordinate system, with respect to the particle

	// for one particle (need to repeat for all particles)
	// Convert observations to map space, to place with respect of the particle
	int i_part = 0;
	for (i_part = 0; i_part < num_particles; ++i_part)
	{
		double x = particles[i_part].x;
		double y = particles[i_part].y;
		double theta = particles[i_part].theta;

		// Create mapped observation list
		std::vector<LandmarkObs> observations_M;

		// Create landmark (in range) list
		std::vector<LandmarkObs> landmark_list_in_range;
		for (unsigned int i = 0; i < map_landmarks.landmark_list.size(); ++i)
		{
			static int counter = 0;
			LandmarkObs landmark;
			landmark.id = map_landmarks.landmark_list[i].id_i;
			landmark.x = map_landmarks.landmark_list[i].x_f;
			landmark.y = map_landmarks.landmark_list[i].y_f;

			if (dist(x, y, landmark.x, landmark.y) <= sensor_range)
			{
				landmark_list_in_range.push_back(landmark);
#ifdef DEBUG_LANDMARK_LIST_IN_RANGE
				if (i_part == DEBUG_PART_NUM)
				{
					std::cout << "LANINRANGE: landmark.x= " << landmark.x << " \t| landmark.y= " << landmark.y << " \t| landmark.id= " << landmark.id << std::endl;
				}
#endif
			}
		}

		// Transform observation coordinates
		for (unsigned int i_obs = 0; i_obs < observations.size(); ++i_obs)
		{
			LandmarkObs observation_M;
			double xM = cos(theta) * observations[i_obs].x - sin(theta) * observations[i_obs].y + x;
			double yM = sin(theta) * observations[i_obs].x + cos(theta) * observations[i_obs].y + y;
			observation_M.x = xM;
			observation_M.y = yM;
			observation_M.id = observations[i_obs].id;
			observations_M.push_back(observation_M);

#ifdef DEBUG_OBSERVATION_MAPPING
			if (i_part == DEBUG_PART_NUM)
			{
				std::cout << "OBS[" << i_obs << "]: observations[i_obs].x = " << observations[i_obs].x << " \t| .y= " << observations[i_obs].y << " \t| .id= " << observations[i_obs].id << std::endl;
				std::cout << "OBS[" << i_obs << "]:       observation_M.x = " << observation_M.x << " \t| .y= " << observation_M.y << " \t| .id= " << observation_M.id << std::endl;
			}
#endif
		}

		ParticleFilter::dataAssociation(landmark_list_in_range, observations_M);

#ifdef DEBUG_DATA_ASSOCIATION
		if (i_part == DEBUG_PART_NUM)
		{
			for (int i_ass = 0; i_ass < observations_M.size(); ++i_ass)
			{
				std::cout << "ASS: observations_M[" << i_ass << "].id= " << observations_M[i_ass].id << " \t| landmark_list_in_range[i_ass].id= " << landmark_list_in_range[i_ass].id << std::endl;
			}
		}
#endif
		particles[i_part].weight = 1;
		weights[i_part] = 1;
		// Now that the landmark and observations are associated, compute the weight
		unsigned int i_obs = 0;
		for (i_obs = 0; i_obs < observations_M.size(); ++i_obs)
		{
			double x_obs = observations_M[i_obs].x;
			double y_obs = observations_M[i_obs].y;
			// find landmark
			unsigned int i_ldmk = 0;
			for (i_ldmk = 0; i_ldmk < landmark_list_in_range.size(); ++i_ldmk)
			{
				if (landmark_list_in_range[i_ldmk].id == observations_M[i_obs].id)
				{
					// Observation corresponds to landmark, update particle weight
					double Pxy;
					double exponent;
					double sig_x = std_landmark[0];
					double sig_y = std_landmark[1];

					double mu_x = landmark_list_in_range[i_ldmk].x;
					double mu_y = landmark_list_in_range[i_ldmk].y;
					exponent = -((x_obs - mu_x) * (x_obs - mu_x) / (2 * sig_x * sig_x) + (y_obs - mu_y) * (y_obs - mu_y) / (2 * sig_y * sig_y));
					Pxy = 1.0 / (2 * M_PI * sig_x * sig_y) * exp(exponent);
#ifdef DEBUG_WEIGHT_CALC
					if (i_part == DEBUG_PART_NUM)
					{
						std::cout << "i_part= " << i_part << " \t| i_obs= " << i_obs << " \t| i_ldmk= " << i_ldmk << " \t| Exp= " << exponent << " \t|Pxy: " << Pxy << std::endl;
					}
#endif
					if (Pxy < EPS)
					{
						Pxy = EPS;
					};
					particles[i_part].weight *= Pxy;
					weights[i_part] *= Pxy;
				}
			}
		}
		//std::cout << "i_obs : " << i_obs << std::endl;
	}
	//std::cout << "i_part : " << i_part << std::endl;
}

void ParticleFilter::resample(void)
{
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::random_device rd;
	std::mt19937 gen(rd());
	std::discrete_distribution<int> d(weights.begin(), weights.end());
	std::vector<Particle> resampled_particles = particles;
	for (int i = 0; i < num_particles; ++i)
	{
		resampled_particles[i] = particles[d(gen)];
	}
	particles = resampled_particles;
}

void ParticleFilter::resampled_test(void)
{

	std::vector<double> probability_vector = {10.50, 50.12, 1.9, 30.4, 30.44};
	std::vector<int> particle_list = {0, 10, 20, 30, 41};
	std::vector<int> resampled_list = particle_list;
		std::random_device rd;
	std::mt19937 gen(rd());
	std::discrete_distribution<int> d(probability_vector.begin(), probability_vector.end());
	std::cout << "Resampled_list: " ;
	for (int i = 0; i < particle_list.size(); ++i)
	{
		resampled_list[i] = particle_list[d(gen)];
		std::cout << resampled_list[i] << " ";
	}
	std::cout << std::endl ;

}

Particle ParticleFilter::SetAssociations(Particle &particle, const std::vector<int> &associations,
										 const std::vector<double> &sense_x, const std::vector<double> &sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	particle.associations = associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;
	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1); // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1); // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1); // get rid of the trailing space
	return s;
}

/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 1000;  // TODO: Set the number of particles
  std::default_random_engine gen;

  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);
  
  is_initialized = 1;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;
  
  for (int i=0; i<num_particles;i++){
    float pred_x;
    float pred_y;
    float pred_theta;
    if (yaw_rate==0){
      pred_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
      pred_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
      pred_theta = particles[i].theta;
    } else {
      pred_x = particles[i].x + velocity/yaw_rate * (sin(particles[i].theta+yaw_rate*delta_t) - sin(particles[i].theta));
      pred_y = particles[i].y + velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta+yaw_rate*delta_t));
      pred_theta = particles[i].theta + yaw_rate*delta_t;
    }
    std::normal_distribution<double> N_x(pred_x,std_pos[0]);  //create normal distribuion around predicted_x  revisit this as perhaps this should be around the sensor inputs?
    std::normal_distribution<double> N_y(pred_y,std_pos[1]);
    std::normal_distribution<double> N_theta(pred_theta,std_pos[2]);
    
    particles[i].x = N_x(gen);          //pick a value from the normal distribution for the new x
    particles[i].y = N_y(gen);          //pick a value from the normal distribution for the new y
    particles[i].theta = N_theta(gen);  //pick a value from the normal distribution for the new theta
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  
  for (unsigned int i=0; i<observations.size();++i){
    
    double closest_obs_range =99999;
    int tempID = -1;

    
    for (unsigned int j=0; j<predicted.size();j++){
      double range = dist(predicted[j].x, predicted[j].y, observations[i].x,observations[i].y);
      if (range<closest_obs_range){
        closest_obs_range = range;
        tempID = predicted[j].id;
      }
    }
    observations[i].id = tempID;
  } 
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  float theta = -M_PI/2;
    
  //vector<int> associations;
  //vector<double> sense_x;
  //vector<double> sense_y;

  // For each particle, P, with observations in vehicle-grid, translate observations to map-grid (particle data stays as estimate)
  for (int p=0; p<num_particles;++p){
    LandmarkObs obsTranslated;                          //translated observations
    vector<LandmarkObs> obsTrans_v;                     //vector of all translated observations for a given particle
    
    for (unsigned int o=0; o<observations.size();o++){
      obsTranslated.x = particles[p].x + (cos(theta) * observations[o].x) - (sin(theta) * observations[o].y);
      obsTranslated.y = particles[p].y + (sin(theta) * observations[o].x) + (cos(theta) * observations[o].x);
      //insert calculation of standard deviation & normal distribution of sensor error
      obsTranslated.id = -1;
      obsTrans_v.push_back(obsTranslated);              //assign to a vector of observations
    }
    
    vector<LandmarkObs> landmarksInRange;
    //Eliminate landmarks that are out of range, sensor_range, for particle p, still iterating through particle by particle
    for (unsigned int l=0; l<map_landmarks.landmark_list.size();l++){
      double landmarkRange = dist(particles[p].x, particles[p].y, map_landmarks.landmark_list[l].x_f,map_landmarks.landmark_list[l].y_f);
      if (landmarkRange<sensor_range){
        landmarksInRange.push_back(LandmarkObs{map_landmarks.landmark_list[l].id_i,map_landmarks.landmark_list[l].x_f,map_landmarks.landmark_list[l].y_f});
      }
    }
    //Associate each observation with the closest landmark
    /*question about associating landmarks with corresponding measurements vs vice versa.  This would decrease the probability
      whereas 1 accurate sensor hit in a cluster of landmarks would result in high probability vs driving it down.
      */
    dataAssociation(landmarksInRange, obsTrans_v);
       
    // Continue with updated probability for this observation
    float gaussProb = 1.0;
    for (unsigned int o=0; o<obsTrans_v.size();++o){
      unsigned int l = 0;
      while(l<landmarksInRange.size()){
        if (landmarksInRange[l].id == obsTrans_v[o].id){
          gaussProb *= multiv_prob(std_landmark[0], std_landmark[1], obsTrans_v[o].x, obsTrans_v[o].y, landmarksInRange[l].x, landmarksInRange[l].y);
          l = landmarksInRange.size();
        } else {
            l++;
        }
      }
        
    }
    if (gaussProb==0){
      gaussProb = 0.000000001;
    }
    particles[p].weight = gaussProb;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::default_random_engine gen;
  std::discrete_distribution<int> distribution(weights.begin(),weights.end());
  vector<Particle> resampled_particles;
  for(int p=0;p<num_particles; p++){
    resampled_particles.push_back(particles[distribution(gen)]);
  }
  particles = resampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

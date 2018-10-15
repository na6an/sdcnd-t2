/*
 * particle_filter.cpp
 *
 *  Author: Tiffany Huang
 * 	Modified by: Nathan
 */

#include <algorithm>
#include <iostream>
#include <iterator>
#include <math.h>
#include <numeric>
#include <random>
#include <sstream>
#include <string>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  default_random_engine gen;
  normal_distribution<double> norm_x(x, std[0]);
  normal_distribution<double> norm_y(y, std[1]);
  normal_distribution<double> norm_theta(theta, std[2]);

  num_particles = 10;

  for (int i = 0; i < num_particles; ++i) {
    Particle particle;

    particle.id = i;
    particle.x = norm_x(gen);
    particle.y = norm_y(gen);
    particle.theta = norm_theta(gen);
    particle.weight = 1.0;

    particles.push_back(particle);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  default_random_engine gen;

  // Update each particle
  for (int i = 0; i < num_particles; ++i) {
    double new_x, new_y, new_theta;
    if (yaw_rate == 0) { // Ensure not divisible by 0
      new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
      new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
      new_theta = particles[i].theta + yaw_rate * delta_t;
    } else {
      new_x =
          particles[i].x + velocity / yaw_rate *
                               (sin(particles[i].theta + yaw_rate * delta_t) -
                                sin(particles[i].theta));
      new_y =
          particles[i].y + velocity / yaw_rate *
                               (cos(particles[i].theta) -
                                cos(particles[i].theta + yaw_rate * delta_t));
      new_theta = particles[i].theta + yaw_rate * delta_t;
    }
    normal_distribution<double> dist_x(new_x, std_pos[0]);
    normal_distribution<double> dist_y(new_y, std_pos[1]);
    normal_distribution<double> dist_theta(new_theta, std_pos[2]);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
                                     std::vector<LandmarkObs> &observations) {
  for (int i = 0; i < (int)observations.size(); ++i) {
    double min_dist = 1E10;
    int min_id = 0;

    for (int j = 0; j < (int)predicted.size(); ++j) {
      double d = dist(observations[i].x, observations[i].y, predicted[j].x,
                      predicted[j].y);
      // Compare distance and update closest
      if (d < min_dist) {
        min_dist = d;
        min_id = predicted[j].id;
      }
    }
    observations[i].id = min_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  double sig_xx = std_landmark[0] * std_landmark[0];
  double sig_yy = std_landmark[1] * std_landmark[1];
  double sig_xy = std_landmark[0] * std_landmark[1];

  // For each particle
  for (int i = 0; i < num_particles; ++i) {
    // Find landmarks within sensor range
    vector<LandmarkObs> predictions;
    for (int j = 0; j < (int)map_landmarks.landmark_list.size(); ++j) {
      double landmark_x = map_landmarks.landmark_list[j].x_f;
      double landmark_y = map_landmarks.landmark_list[j].y_f;
      int landmark_id = map_landmarks.landmark_list[j].id_i;

      if ((landmark_x - particles[i].x) < sensor_range &&
          (landmark_y - particles[i].y) < sensor_range) {
        predictions.push_back(LandmarkObs{landmark_id, landmark_x, landmark_y});
      }
    }

    // Transform to map coordinate system
    vector<LandmarkObs> obs;
    double theta = particles[i].theta;
    for (int j = 0; j < (int)observations.size(); ++j) {
      double x_map = particles[i].x + (cos(theta) * observations[j].x) -
                     (sin(theta) * observations[j].y);
      double y_map = particles[i].y + (sin(theta) * observations[j].x) +
                     (cos(theta) * observations[j].y);
      obs.push_back(LandmarkObs{observations[j].id, x_map, y_map});
    }

    // Associate observations to landmarks
    dataAssociation(predictions, obs);

    vector<int> associations;
    vector<double> sense_x, sense_y;

    particles[i].weight = 1.0;
    for (int j = 0; j < (int)obs.size(); ++j) {
      int id_associated = obs[j].id;
      double mu_x, mu_y;
      for (int k = 0; k < (int)predictions.size(); k++) {
        if (predictions[k].id == id_associated) {
          mu_x = predictions[k].x;
          mu_y = predictions[k].y;
        }
      }

      // Update weight
      particles[i].weight *= exp(-(pow(obs[j].x - mu_x, 2) / (2 * sig_xx) +
                                   (pow(obs[j].y - mu_y, 2) / (2 * sig_yy)))) /
                             (2 * M_PI * sig_xy);

      associations.push_back(id_associated);
      sense_x.push_back(obs[j].x);
      sense_y.push_back(obs[j].y);
    }

    // Visualize
    particles[i] =
        SetAssociations(particles[i], associations, sense_x, sense_y);
  }
}

void ParticleFilter::resample() {
  // Update weights
  weights.clear();
  for (int i = 0; i < num_particles; ++i) {
    weights.push_back(particles[i].weight);
  }

  // Resample particles
  default_random_engine gen;
  discrete_distribution<int> distrib(weights.begin(), weights.end());

  vector<Particle> new_particles;
  new_particles.resize(num_particles);
  for (int i = 0; i < num_particles; ++i) {
    int j = distrib(gen);
    //	int j = int(rand() * num_particles);
    new_particles[i] = particles[j];
  }
  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle &particle,
                                         const std::vector<int> &associations,
                                         const std::vector<double> &sense_x,
                                         const std::vector<double> &sense_y) {
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;

  return particle;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

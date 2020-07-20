#!/usr/bin/env python
from numpy.random import choice
import numpy as np
from random import randint
from math import sqrt
import cv2
from matplotlib import pyplot
from scipy.ndimage import gaussian_filter

GAUSSIAN_EXPANSION = 2
DISTANCE_ERROR = 0.3
ORIENTATION_ERROR = 0.5

class ParticleLocalization:
    def __init__(self, map_img, map_resolution, n_particles, fixed_angle=False):
        self.map_img = map_img
        self.map_prob = self.create_probability_map()
        self.map_resolution = map_resolution
        self.n_particles = n_particles
        self.particles = []
        # self.particles = self.create_particles(n_particles,fixed_angle)
        # print("plotting map")
        # pyplot.matshow(self.map_prob)
        # pyplot.show()
        
    def create_probability_map(self):
        map_prob = np.zeros(self.map_img.shape)
        map_image = np.copy(self.map_img)
        map_image[map_image == 19] = 0
        map_prob = cv2.normalize(gaussian_filter(map_image, GAUSSIAN_EXPANSION), map_prob, 0, 100, cv2.NORM_MINMAX)+np.ones(self.map_img.shape)
        return map_prob

    def MCL(self, movement, sensor_data):
        new_particles = [] # Se guardan aqui las particulas una vez trasladadas y calculando sus nuevos pesos
        weights = []

        ## Aplicar el traslado a todas las particulas
        ## Aplicar el modelo de sensor para obtener nuevos pesos
        for particle in self.particles:
            particle = self.particle_motion_model(movement, particle)
            if particle != None:
                new_particles.append(particle)
                weight = self.likelihood_field_model(sensor_data, particle)
                weights.append(weight)
        ## Samplear las nuevas particulas
        new_particles = np.array(new_particles)
        if sum(weights) != 0:
            weights = np.array(weights).astype(np.float32)/sum(weights) # Hace que la suma de los pesos sea 1
            indexes = choice(new_particles.shape[0], self.n_particles, p=weights, replace=True)
            sampled_particles = new_particles[indexes]
            return sampled_particles
        return new_particles

    def particle_motion_model(self, movement, particle):
        dist = sqrt(movement[0]**2 + movement[1]**2)/self.map_resolution
        dist_x = dist*np.cos(particle[2])
        dist_y = dist*np.sin(particle[2])
        x = int(particle[0] + dist_x + np.random.normal(0,3))
        y = int(particle[1] + dist_y + np.random.normal(0,3))
        theta = particle[2] + movement[2] + np.random.normal(0,0.01)
        new_particle = [x,y,theta]
        if x >= self.map_img.shape[0] or x < 0 or y >= self.map_img.shape[1] or y < 0:
            return None
        elif self.map_img[x,y] != 0:
            return None
        return new_particle
        # Retorna la lista actualizada de particulas

    def likelihood_field_model(self, sensor_data, particle):
        x = particle[0]
        y = particle[1]
        theta = particle[2]
        q = 1
        for k in range(0,len(sensor_data),5):
            if sensor_data[k] != 4:
                z_k = sensor_data[k]/self.map_resolution
                theta_sens = (k-29)*np.pi/180
                x_z = int(x + z_k*np.cos(theta+theta_sens))
                y_z = int(y + z_k*np.sin(theta+theta_sens))
                if x_z >= self.map_img.shape[0] or y_z >= self.map_img.shape[1] or x_z < 0 or y_z < 0:
                    prob = 0
                else:
                    prob = self.map_prob[x_z,y_z]/10
                q = q*prob
        return q
        # Evalua la informacion del sensor y retorna las probabilidades de ocurrencia para cada particula

    def create_particles(self, n_particles, fixed_angle=False):
        particles = []
        h,w = self.map_img.shape
        count = 0
        while count < n_particles:
            x = randint(0,h-1)
            y = randint(0,w-1)
            theta = randint(0,359)*np.pi/180 if not fixed_angle else np.pi/2
            if self.map_img[x,y] == 0:
                particles.append([x,y,theta])
                count += 1
        return particles
        # Retorna una lista de particulas distribuidas uniformemente

    def create_particles_from_pose(self, pose):
        particles = []
        h,w = self.map_img.shape
        pose_y = pose[0]/self.map_resolution
        pose_x = h - pose[1]/self.map_resolution
        pose_theta = pose[2] - np.pi/2
        count = 0
        while count < self.n_particles:
            dist_error = DISTANCE_ERROR/self.map_resolution
            x = int(pose_x + np.random.normal(0,DISTANCE_ERROR/(5*self.map_resolution)))
            y = int(pose_y + np.random.normal(0,DISTANCE_ERROR/(5*self.map_resolution)))
            theta = pose_theta + np.random.normal(ORIENTATION_ERROR/5)
            if self.map_img[x,y] == 0:
                particles.append([x,y,theta])
                count+=1
        self.particles = particles

    def fit_particles(self):
        std = np.std(self.particles, axis=0)
        if std[0] < 8 and std[1] < 8 and std[2] < 5:
            return True
        return False

    def show_map(self):
        img = np.copy(self.map_img)
        for particle in self.particles:
            img[particle[0],particle[1]] = 50
        pyplot.matshow(img)
        pyplot.show()

    def test_sensor_model(self, sensor_data):
        sensor_data = np.array(sensor_data)
        img = np.zeros(self.map_img.shape)
        for particle in self.particles:
            img[particle[0],particle[1]] = self.likelihood_field_model(sensor_data,particle)
        norm_img = np.zeros(self.map_img.shape)
        norm_img = cv2.normalize(img, norm_img, 0, 100, cv2.NORM_MINMAX)
        
        print("Finished")
        pyplot.matshow(self.map_img + norm_img)
        pyplot.show()

    def run_MCL(self, movement, sensor_data):
        self.particles = self.MCL(movement, sensor_data)


    def test_MCL(self, sensor_data):
        partic = self.MCL(None, sensor_data)
        img = np.copy(self.map_img)
        for particle in partic:
            img[int(particle[0]),int(particle[1])] = 50
        pyplot.matshow(img)
        pyplot.show()

    def test_motion_model(self, movement):
        new_particles = []
        for particle in self.particles:
            particle = self.particle_motion_model(movement, particle)
            if particle != None:
                new_particles.append(particle)
        self.particles = new_particles
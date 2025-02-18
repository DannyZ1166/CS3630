from grid import *
from particle import Particle
from utils import *
import setting
import numpy as np
np.random.seed(setting.RANDOM_SEED)
from itertools import product
from typing import List, Tuple


def create_random(count: int, grid: CozGrid) -> List[Particle]:
    """
    Returns a list of <count> random Particles in free space.

    Parameters:
        count: int, the number of random particles to create
        grid: a Grid, passed in to motion_update/measurement_update
            see grid.py for definition

    Returns:
        List of Particles with random coordinates in the grid's free space.
    """
    # TODO: implement here
    # -------------------
    particles = []
    while len(particles) < count:
        x = random.randint(0, grid.width - 1)
        y = random.randint(0, grid.height - 1)
        if grid.is_in(x, y) and grid.is_free(x, y):
            h = random.uniform(0, 360)
            particles.append(Particle(x, y, h))
    return particles
    # -------------------
    

# ------------------------------------------------------------------------
def motion_update(old_particles:  List[Particle], odometry_measurement: Tuple, grid: CozGrid) -> List[Particle]:
    """
    Implements the motion update step in a particle filter. 
    Refer to setting.py and utils.py for required functions and noise parameters
    For more details, please read "Motion update details" section and Figure 3 in "CS3630_Project2_Spring_2025.pdf"


    NOTE: the GUI will crash if you have not implemented this method yet. To get around this, try setting new_particles = old_particles.

    Arguments:
        old_particles: List 
            list of Particles representing the belief before motion update p(x_{t-1} | u_{t-1}) in *global coordinate frame*
        odometry_measurement: Tuple
            noisy estimate of how the robot has moved since last step, (dx, dy, dh) in *local robot coordinate frame*

    Returns: 
        a list of NEW particles representing belief after motion update \tilde{p}(x_{t} | u_{t})
    """
    new_particles = []

    for particle in old_particles:
        # extract the x/y/heading rom the particle
        x_g, y_g, h_g = particle.xyh
        # and the change in x/y/heading from the odometry measurement
        dx_r, dy_r, dh_r = odometry_measurement

        new_particle = None
        # TODO: implement here
        # ----------------------------------
        # align odometry_measurement's robot frame coords with particle's global frame coords (heading already aligned)
        dx_global, dy_global = rotate_point(dx_r, dy_r, h_g)
        dx_global_noisy = add_gaussian_noise(dx_global, setting.ODOM_TRANS_SIGMA)
        dy_global_noisy = add_gaussian_noise(dy_global, setting.ODOM_TRANS_SIGMA)
        dh_noisy = add_gaussian_noise(dh_r, setting.ODOM_HEAD_SIGMA)

        # compute estimated new coordinate, using current pose and odometry measurements. Make sure to add noise to simulate the uncertainty in the robot's movement.
        new_x = x_g + dx_global_noisy
        new_y = y_g + dy_global_noisy
        new_h = (h_g + dh_noisy) % 360
        # create a new particle with this noisy coordinate
        new_particle = Particle(new_x, new_y, new_h)
        # ----------------------------------
        new_particles.append(new_particle)

    return new_particles

# ------------------------------------------------------------------------
def generate_marker_pairs(robot_marker_list: List[Tuple], particle_marker_list: List[Tuple]) -> List[Tuple]:
    """ Pair markers in order of closest distance

        Arguments:
        robot_marker_list -- List of markers observed by the robot: [(x1, y1, h1), (x2, y2, h2), ...]
        particle_marker_list -- List of markers observed by the particle: [(x1, y1, h1), (x2, y2, h2), ...]

        Returns: List[Tuple] of paired robot and particle markers: [((xp1, yp1, hp1), (xr1, yr1, hr1)), ((xp2, yp2, hp2), (xr2, yr2, hr2),), ...]
    """
    marker_pairs = []
    while len(robot_marker_list) > 0 and len(particle_marker_list) > 0:
        # TODO: implement here
        # ----------------------------------
        marker_pairs = []
    robot_markers = robot_marker_list.copy()
    particle_markers = particle_marker_list.copy()
    while robot_markers and particle_markers:
        best_dist = float('inf')
        best_robot = None
        best_particle = None
        # find the (particle marker,robot marker) pair with shortest grid distance
        for rm in robot_markers:
            for pm in particle_markers:
                d = grid_distance(rm[0], rm[1], pm[0], pm[1])
                if d < best_dist:
                    best_dist = d
                    best_robot = rm
                    best_particle = pm
        if best_robot is None or best_particle is None:
            break
        marker_pairs.append((best_robot, best_particle))
        # add this pair to marker_pairs and remove markers from corresponding lists
        robot_markers.remove(best_robot)
        particle_markers.remove(best_particle)
        # ----------------------------------
        pass
    return marker_pairs

# ------------------------------------------------------------------------
def marker_likelihood(robot_marker: Tuple, particle_marker: Tuple) -> float:
    """ Calculate likelihood of reading this marker using Gaussian PDF. 
        The standard deviation of the marker translation and heading distributions 
        can be found in setting.py
        
        Some functions in utils.py might be useful in this section

        Arguments:
        robot_marker -- Tuple (x,y,theta) of robot marker pose
        particle_marker -- Tuple (x,y,theta) of particle marker pose

        Returns: float probability
    """
    l = 0.0
    # TODO: implement here
    # ----------------------------------
    # find the distance between the particle marker and robot marker
    dx = robot_marker[0] - particle_marker[0]
    dy = robot_marker[1] - particle_marker[1]
    distance = math.sqrt(dx**2 + dy**2)
    trans_likelihood = (1.0 / (math.sqrt(2 * math.pi) * setting.MARKER_TRANS_SIGMA)) * math.exp(- (distance ** 2) / (2 * setting.MARKER_TRANS_SIGMA ** 2))
    
    # find the difference in heading between the particle marker and robot marker
    dh = diff_heading_deg(robot_marker[2], particle_marker[2])
    rot_likelihood = (1.0 / (math.sqrt(2 * math.pi) * setting.MARKER_HEAD_SIGMA)) * math.exp(- (dh ** 2) / (2 * setting.MARKER_HEAD_SIGMA ** 2))

    # calculate the likelihood of this marker using the gaussian pdf. You can use the formula on Page 5 of "CS3630_Project2_Spring_2025.pdf"
    likelihood = trans_likelihood * rot_likelihood
    return likelihood
    # ----------------------------------
    return l

# ------------------------------------------------------------------------
def particle_likelihood(robot_marker_list: List[Tuple], particle_marker_list: List[Tuple]) -> float:
    """ Calculate likelihood of the particle pose being the robot's pose

        Arguments:
        robot_marker_list -- List of markers (x,y,theta) observed by the robot
        particle_marker_list -- List of markers (x,y,theta) observed by the particle

        Returns: float probability
    """
    l = 1.0
    marker_pairs = generate_marker_pairs(robot_marker_list, particle_marker_list)
    # TODO: implement here
    # ----------------------------------
    # update the particle likelihood using the likelihood of each marker pair
    # HINT: consider what the likelihood should be if there are no pairs generated
    if not marker_pairs:
        return 1e-3

    likelihood = 1.0
    for (robot_marker, particle_marker) in marker_pairs:
        likelihood *= marker_likelihood(robot_marker, particle_marker)
        
    return likelihood
    
    # ----------------------------------
    return l

# ------------------------------------------------------------------------
def measurement_update(particles: List[Particle], measured_marker_list: List[Tuple], grid: CozGrid) -> List[Particle]:
    """ Particle filter measurement update
       
        NOTE: the GUI will crash if you have not implemented this method yet. To get around this, try setting measured_particles = particles.
        
        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before measurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update

    """
    
    measuredMarkerListLength, num, weights = len(measured_marker_list), 0, []
    for particle in particles:
        if measuredMarkerListLength:
            robotMarkers = particle.read_markers(grid=grid)
            markersLength = len(robotMarkers)
            if not markersLength:
                weights.append(0)
            elif not grid.is_free(x=particle.x, y=particle.y):
                weights.append(0)
                num += 1
            else:
                lst, confidence = [], 1
                for measured_marker in measured_marker_list:
                    if markersLength:
                        worst = min(robotMarkers, key=lambda m: grid_distance(x1=measured_marker[0], y1=measured_marker[1], x2=m[0], y2=m[1]))
                        robotMarkers.remove(worst)
                        markersLength = len(robotMarkers)
                        lst.append((measured_marker, worst))
                for marker, worst in lst:
                    confidence *= math.exp(-1 * (math.pow(grid_distance(marker[0], marker[1], worst[0], worst[1]), 2) / 
                                  (2 * math.pow(setting.MARKER_TRANS_SIGMA, 2)) + 
                                  math.pow(diff_heading_deg(marker[2], worst[2]), 2) / 
                                  (2 * math.pow(setting.MARKER_HEAD_SIGMA, 2))))
                weights.append(confidence)
    particleLength, weightsSum = len(particles), sum(weights)
    if measuredMarkerListLength < 1 or weightsSum == 0:
        weights = particleLength * [(1 / float(particleLength))]
    else:
        weights = [weight / weightsSum for weight in weights]
    beliefList = [Particle(x=particle.x, y=particle.y, heading=particle.h) 
                  for particle in np.random.choice(particles, size=(particleLength - min(particleLength, 50 + num)), p=weights).tolist()]
    for x, y in [grid.random_free_place() for i in range(min(particleLength, 50 + num))]:
        beliefList.append(Particle(x=x, y=y))
    return beliefList



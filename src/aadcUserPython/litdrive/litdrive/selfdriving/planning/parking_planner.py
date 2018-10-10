# Imports
import math
import numpy as np
import matplotlib.pyplot as plt


# ---------------------------- Code by Stefan Rader -------------------------------------
class ParkingTrajectoryGenerator:
    # Class Variables

    # Vehicle Parameters
    __l = 0.356  # length between front and rear axle in m
    __b = 0.37  # width of car in m
    __l_1 = 0.12  # length between front axle and bumper in m
    __l_2 = 0.108  # length between rear axle and bumper in m
    __alpha_max = math.radians(45)  # maximum steering angle in rad
    # alpha_c = alpha_max # constant steering angle in rad
    __rho_min = 1 / math.tan(__alpha_max)  # radius of the turning cycle of the car in m

    # Driving lane and parking spot parameters
    __h_cd = 0.974 - 2 * 0.03  # width of driving lane in m
    __h_pd = (0.96 - 3 * 0.02) / 2  # width of parking space in m
    __h_pw = 0.85  # depth of parking space in m
    __h_ps = (__h_pd - __b) / 2  # = h_pr = h_pl = h_ps -> for symmetrical parking -> space between car and parking space boundaries in m

    # Parameters for calculation of the Trajectory Points
    __num_points_per_segment = 100
    __pull_out_left_straight_offset = 0.2
    __r_B2 = math.sqrt((__l + __l_1) ** 2 + (__rho_min + __b / 2) ** 2)
    __s_m = -math.sqrt((__rho_min - __b / 2) ** 2 - (__rho_min - __h_pd / 2) ** 2)
    __s_max = __h_cd - __r_B2
    __s = max(abs(__s_m), abs(__s_max))

    # Points of Parking Trajectory
    __parkingTrajectoryPoints_x_rear_axle = np.zeros(2 * __num_points_per_segment)
    __parkingTrajectoryPoints_y_rear_axle = np.zeros(2 * __num_points_per_segment)
    # __parkingTrajectoryPoints_x_front_axle = np.zeros(2*__num_points_per_segment)
    # __parkingTrajectoryPoints_y_front_axle = np.zeros(2*__num_points_per_segment)
    __pullOutLeftTrajectoryPoints_x_rear_axle = np.zeros(2 * __num_points_per_segment)
    __pullOutLeftTrajectoryPoints_y_rear_axle = np.zeros(2 * __num_points_per_segment)
    # __pullOutLeftTrajectoryPoints_x_front_axle = np.zeros(2*__num_points_per_segment)
    # __pullOutLeftTrajectoryPoints_y_front_axle = np.zeros(2*__num_points_per_segment)

    # Heading of Parking Trajectory
    __parkingTrajectoryHeading_rear_axle = np.zeros(2 * __num_points_per_segment)

    # Parameter for Representing Circle Arc as Polynomial (Bezier)
    __c = 0.55191502449

    # Parameters of Steering Angle Controller (Saturated Control) from Paper
    __K_t = 8
    __K = 5.85
    __a_0 = 0.17
    __u = np.tan(__alpha_max) / __l

    # Vehicle Heading for test purposes (idealised)
    __theta = np.zeros(2 * __num_points_per_segment)

    # Constructor
    def __init__(self, targetParkingSpot_x=0, targetParkingSpot_y=0):
        self.__targetPoint_x_rear_axle = targetParkingSpot_x + self.__h_pw - self.__l_2 - self.__h_ps
        self.__targetPoint_y_rear_axle = targetParkingSpot_y
        self.__targetPoint_x_front_axle = targetParkingSpot_x + self.__h_pw - self.__l_2 - self.__h_ps - self.__l
        self.__targetPoint_y_front_axle = targetParkingSpot_y
        self.calcParkingTrajectory()
        self.calcPullOutLeftTrajectory()

    # Setter
    def setTargetParkingSpot(self, targetParkingSpot_x=0, targetParkingSpot_y=0):
        self.__targetPoint_x_rear_axle = targetParkingSpot_x + self.__h_pw - self.__l_2 - self.__h_ps
        self.__targetPoint_y_rear_axle = targetParkingSpot_y
        self.__targetPoint_x_front_axle = targetParkingSpot_x + self.__h_pw - self.__l_2 - self.__h_ps - self.__l
        self.__targetPoint_y_front_axle = targetParkingSpot_y
        self.calcParkingTrajectory()
        self.calcPullOutLeftTrajectory()

    # Getter
    def getParkingStartPoint(self):
        return self.__parkingTrajectoryPoints_x_rear_axle[-1], self.__parkingTrajectoryPoints_y_rear_axle[-1]

    def getParkingEndPoint(self):
        return self.__targetPoint_x_rear_axle, self.__targetPoint_y_rear_axle

    def getParkingTrajectoryPolynomials(self):
        return self.__parkingTrajectory_polynomial_coefficients_circle_arc_x, self.__parkingTrajectory_polynomial_coefficients_circle_arc_y, self.__parkingTrajectory_polynomial_coefficients_straight_x, self.__parkingTrajectory_polynomial_coefficients_straight_y

    def gePullOutLeftTrajectoryPolynomials(self):
        return self.__pullOutLeftTrajectory_polynomial_coefficients_circle_arc_x, self.__pullOutLeftTrajectory_polynomial_coefficients_circle_arc_y, self.__pullOutLeftTrajectory_polynomial_coefficients_straight_x, self.__pullOutLeftTrajectory_polynomial_coefficients_straight_y

    # Functions
    def calcParkingTrajectory(self):
        # = Pull Out Right Trajectory
        # Target Point Rear End of the Parking Spot (Rear end of the axle)
        S_x_rear_axle = self.__targetPoint_x_rear_axle - self.__h_pw + self.__l_2 + self.__h_ps + self.__s
        S_y_rear_axle = self.__targetPoint_y_rear_axle
        # S_x_front_axle = self.targetPoint_x_front_axle - self.h_pw + self.l_2 + self.h_ps + self.s + self.l
        # S_y_front_axle = self.targetPoint_y_front_axle
        O_x_rear_axle = S_x_rear_axle
        O_y_rear_axle = S_y_rear_axle + self.__rho_min
        # O_x_front_axle = S_x_front_axle
        # O_y_front_axle = S_y_front_axle + self.rho_min

        # Points on Unit circle with Origin O
        P_0_circle_arc_x = O_x_rear_axle
        P_0_circle_arc_y = O_y_rear_axle - 1
        P_1_circle_arc_x = O_x_rear_axle - self.__c
        P_1_circle_arc_y = O_y_rear_axle - 1
        P_2_circle_arc_x = O_x_rear_axle - 1
        P_2_circle_arc_y = O_y_rear_axle - self.__c
        P_3_circle_arc_x = O_x_rear_axle - 1
        P_3_circle_arc_y = O_y_rear_axle

        # Polynomial of the circle arc
        self.__parkingTrajectory_polynomial_coefficients_circle_arc_x = np.poly1d(
            [self.__rho_min * (P_3_circle_arc_x + 3. * P_1_circle_arc_x - 3. * P_2_circle_arc_x - P_0_circle_arc_x),
             self.__rho_min * 3 * (P_2_circle_arc_x - 2 * P_1_circle_arc_x + P_0_circle_arc_x),
             self.__rho_min * 3 * (P_1_circle_arc_x - P_0_circle_arc_x), self.__rho_min * P_0_circle_arc_x])
        self.__parkingTrajectory_polynomial_coefficients_circle_arc_y = np.poly1d(
            [self.__rho_min * (P_3_circle_arc_y + 3. * P_1_circle_arc_y - 3. * P_2_circle_arc_y - P_0_circle_arc_y),
             self.__rho_min * 3 * (P_2_circle_arc_y - 2 * P_1_circle_arc_y + P_0_circle_arc_y),
             self.__rho_min * 3 * (P_1_circle_arc_y - P_0_circle_arc_y), self.__rho_min * P_0_circle_arc_y])

        # Polynomial of the straight
        self.__parkingTrajectory_polynomial_coefficients_straight_x = np.poly1d(
            [0, 0, S_x_rear_axle - self.__targetPoint_x_rear_axle, self.__targetPoint_x_rear_axle])
        self.__parkingTrajectory_polynomial_coefficients_straight_y = np.poly1d(
            [0, 0, S_y_rear_axle - self.__targetPoint_y_rear_axle, self.__targetPoint_y_rear_axle])

        self.__parkingTrajectoryPoints_x_rear_axle[: self.__num_points_per_segment] = np.linspace(
            self.__targetPoint_x_rear_axle, S_x_rear_axle, self.__num_points_per_segment)
        self.__parkingTrajectoryPoints_y_rear_axle[: self.__num_points_per_segment] = np.ones(
            self.__num_points_per_segment) * self.__targetPoint_y_rear_axle
        # self.__parkingTrajectoryHeading_rear_axle[ : self.__num_points_per_segment] = np.ones(self.__num_points_per_segment)*math.pi
        # self.parkingTrajectoryPoints_x_front_axle[0 : self.num_points_per_segment] = np.linspace(self.targetPoint_x_front_axle, S_x_front_axle, self.num_points_per_segment)
        # self.parkingTrajectoryPoints_y_front_axle[0 : self.num_points_per_segment] = np.ones(self.num_points_per_segment)*self.targetPoint_y_front_axle
        circle_arc_angle = np.linspace(math.pi, math.pi * (3 / 2), self.__num_points_per_segment)
        # heading_angle = np.linspace(math.pi, math.pi/2, self.__num_points_per_segment)

        # Vehicle Heading for test
        self.__theta[: self.__num_points_per_segment] = math.pi
        self.__theta[self.__num_points_per_segment:] = np.linspace(math.pi, math.pi / 2, self.__num_points_per_segment)

        # i = self.__num_points_per_segment
        # for angle in circle_arc_angle :
        self.__parkingTrajectoryPoints_x_rear_axle[self.__num_points_per_segment:] = self.__rho_min * np.cos(
            circle_arc_angle) + O_x_rear_axle
        self.__parkingTrajectoryPoints_y_rear_axle[self.__num_points_per_segment:] = self.__rho_min * np.sin(
            circle_arc_angle) + O_y_rear_axle

        # self.__parkingTrajectoryPoints_x_front_axle[ : self.__num_points_per_segment] = self.__parkingTrajectoryPoints_x_rear_axle[ : self.__num_points_per_segment] - self.__l
        # self.__parkingTrajectoryPoints_y_front_axle[ : self.__num_points_per_segment] = self.__parkingTrajectoryPoints_y_rear_axle[ : self.__num_points_per_segment]
        # self.__parkingTrajectoryPoints_x_front_axle[self.__num_points_per_segment : ] = self.__parkingTrajectoryPoints_x_rear_axle[self.__num_points_per_segment : ] + np.cos(self.__theta[self.__num_points_per_segment : ])*self.__l
        # self.__parkingTrajectoryPoints_y_front_axle[self.__num_points_per_segment : ] = self.__parkingTrajectoryPoints_y_rear_axle[self.__num_points_per_segment : ] + np.sin(self.__theta[self.__num_points_per_segment : ])*self.__l

        # self.__parkingTrajectoryHeading_rear_axle[self.__num_points_per_segment : ] = heading_angle
        # self.parkingTrajectoryPoints_x_front_axle[i] = self.rho_min*math.cos(angle) + O_x_front_axle
        # self.parkingTrajectoryPoints_y_front_axle[i] = self.rho_min*math.sin(angle) + O_y_front_axle
        #    i += 1

        # Printing
        # t = np.linspace(0, 1, 100)

        # poly_circle_arc_x = self.__parkingTrajectory_polynomial_coefficients_circle_arc_x(t)
        # poly_circle_arc_y = self.__parkingTrajectory_polynomial_coefficients_circle_arc_y(t)

        # poly_straight_x = self.__parkingTrajectory_polynomial_coefficients_straight_x(t)
        # poly_straight_y = self.__parkingTrajectory_polynomial_coefficients_straight_y(t)

        # plt.plot(self.__parkingTrajectoryPoints_x_rear_axle, self.__parkingTrajectoryPoints_y_rear_axle, 'b.')
        # plt.plot(poly_circle_arc_x, poly_circle_arc_y, 'r.')
        # plt.plot(poly_straight_x, poly_straight_y, 'r.')
        # plt.show()

        # plt.stem(self.__parkingTrajectoryHeading_rear_axle)
        # plt.show()

        return self.__parkingTrajectory_polynomial_coefficients_circle_arc_x, self.__parkingTrajectory_polynomial_coefficients_circle_arc_y, self.__parkingTrajectory_polynomial_coefficients_straight_x, self.__parkingTrajectory_polynomial_coefficients_straight_y

    def calcPullOutLeftTrajectory(self):
        # Target Point Rear End of the Parking Spot (Rear end of the axle)
        S_x_rear_axle = self.__targetPoint_x_rear_axle - self.__h_pw + self.__l_2 + self.__h_ps + self.__s - self.__pull_out_left_straight_offset
        S_y_rear_axle = self.__targetPoint_y_rear_axle
        # S_x_front_axle = self.targetPoint_x_front_axle - self.h_pw + self.l_2 + self.h_ps + self.s + self.l
        # S_y_front_axle = self.targetPoint_y_front_axle
        O_x_rear_axle = S_x_rear_axle
        O_y_rear_axle = S_y_rear_axle - self.__rho_min
        # O_x_front_axle = S_x_front_axle
        # O_y_front_axle = S_y_front_axle + self.rho_min

        # Points on Unit circle with Origin O
        P_0_circle_arc_x = O_x_rear_axle - 1
        P_0_circle_arc_y = O_y_rear_axle
        P_1_circle_arc_x = O_x_rear_axle - 1
        P_1_circle_arc_y = O_y_rear_axle + self.__c
        P_2_circle_arc_x = O_x_rear_axle - self.__c
        P_2_circle_arc_y = O_y_rear_axle + 1
        P_3_circle_arc_x = O_x_rear_axle
        P_3_circle_arc_y = O_y_rear_axle + 1

        # Polynomial of the circle arc
        self.__pullOutLeftTrajectory_polynomial_coefficients_circle_arc_x = np.poly1d(
            [self.__rho_min * (P_3_circle_arc_x + 3. * P_1_circle_arc_x - 3. * P_2_circle_arc_x - P_0_circle_arc_x),
             self.__rho_min * 3 * (P_2_circle_arc_x - 2 * P_1_circle_arc_x + P_0_circle_arc_x),
             self.__rho_min * 3 * (P_1_circle_arc_x - P_0_circle_arc_x), self.__rho_min * P_0_circle_arc_x])
        self.__pullOutLeftTrajectory_polynomial_coefficients_circle_arc_y = np.poly1d(
            [self.__rho_min * (P_3_circle_arc_y + 3. * P_1_circle_arc_y - 3. * P_2_circle_arc_y - P_0_circle_arc_y),
             self.__rho_min * 3 * (P_2_circle_arc_y - 2 * P_1_circle_arc_y + P_0_circle_arc_y),
             self.__rho_min * 3 * (P_1_circle_arc_y - P_0_circle_arc_y), self.__rho_min * P_0_circle_arc_y])

        # Polynomial of the straight
        self.__pullOutLeftTrajectory_polynomial_coefficients_straight_x = np.poly1d(
            [0, 0, S_x_rear_axle - self.__targetPoint_x_rear_axle, self.__targetPoint_x_rear_axle])
        self.__pullOutLeftTrajectory_polynomial_coefficients_straight_y = np.poly1d(
            [0, 0, S_y_rear_axle - self.__targetPoint_y_rear_axle, self.__targetPoint_y_rear_axle])

        self.__pullOutLeftTrajectoryPoints_x_rear_axle[0: self.__num_points_per_segment] = np.linspace(
            self.__targetPoint_x_rear_axle, S_x_rear_axle, self.__num_points_per_segment)
        self.__pullOutLeftTrajectoryPoints_y_rear_axle[0: self.__num_points_per_segment] = np.ones(
            self.__num_points_per_segment) * self.__targetPoint_y_rear_axle
        # self.parkingTrajectoryPoints_x_front_axle[0 : self.num_points_per_segment] = np.linspace(self.targetPoint_x_front_axle, S_x_front_axle, self.num_points_per_segment)
        # self.parkingTrajectoryPoints_y_front_axle[0 : self.num_points_per_segment] = np.ones(self.num_points_per_segment)*self.targetPoint_y_front_axle
        circle_arc_angle = np.linspace(math.pi, math.pi / 2, self.__num_points_per_segment)

        i = self.__num_points_per_segment
        for angle in circle_arc_angle:
            self.__pullOutLeftTrajectoryPoints_x_rear_axle[i] = self.__rho_min * np.cos(angle) + O_x_rear_axle
            self.__pullOutLeftTrajectoryPoints_y_rear_axle[i] = self.__rho_min * np.sin(angle) + O_y_rear_axle
            # self.parkingTrajectoryPoints_x_front_axle[i] = self.rho_min*math.cos(angle) + O_x_front_axle
            # self.parkingTrajectoryPoints_y_front_axle[i] = self.rho_min*math.sin(angle) + O_y_front_axle
            i += 1

        # Printing
        # t = np.linspace(0, 1, 100)

        # poly_circle_arc_x = self.__pullOutLeftTrajectory_polynomial_coefficients_circle_arc_x(t)
        # poly_circle_arc_y = self.__pullOutLeftTrajectory_polynomial_coefficients_circle_arc_y(t)

        # poly_straight_x = self.__pullOutLeftTrajectory_polynomial_coefficients_straight_x(t)
        # poly_straight_y = self.__pullOutLeftTrajectory_polynomial_coefficients_straight_y(t)

        # plt.plot(self.__parkingTrajectoryPoints_x_rear_axle, self.__parkingTrajectoryPoints_y_rear_axle, 'b.')
        # plt.plot(self.__pullOutLeftTrajectoryPoints_x_rear_axle, self.__pullOutLeftTrajectoryPoints_y_rear_axle, 'b.')
        # plt.plot(poly_circle_arc_x, poly_circle_arc_y, 'r.')
        # plt.plot(poly_straight_x, poly_straight_y, 'r.')
        # plt.show()

        return self.__pullOutLeftTrajectory_polynomial_coefficients_circle_arc_x, self.__pullOutLeftTrajectory_polynomial_coefficients_circle_arc_y, self.__pullOutLeftTrajectory_polynomial_coefficients_straight_x, self.__pullOutLeftTrajectory_polynomial_coefficients_straight_y

    def getSteeringAngle(self, actualPoint_y, vehicle_heading):
        theta = vehicle_heading - math.pi
        print(theta)

        v = self.__K * (theta - self.__a_0 * actualPoint_y)
        alpha = np.arctan(self.__l * self.__u * np.tanh(self.__K_t * v))
        return alpha


# ---------------------------- Code by Stefan Gasser -------------------------------------
class ParkingPlanner:

    inv_mat = np.array([[-1, 0, 0, 0], [3, 1, 0, 0], [-3, -2, -1, 0], [1, 1, 1, 1]])

    def __init__(self):
        self.gen = ParkingTrajectoryGenerator()

    def __invert_poly(self, poly):
        # -1
        #  3  1
        # -3 -2 -1
        #  1  1  1  1
        return np.poly1d(self.inv_mat@np.array([poly[t] for t in range(4)][::-1]))

    @staticmethod
    def __move_poly(poly, offset=0):
        return poly + np.poly1d([offset])

    def __rotate_poly(self, poly_x, poly_y, theta, x=0, y=0):
        # TODO copy code from slack by stefan
        c = self.__move_poly(poly_x, -x)
        d = self.__move_poly(poly_y, -y)
        xy = get_rot_mat(theta) @ np.array([[c[t] for t in range(4)][::-1], [d[t] for t in range(4)][::-1]])
        return self.__move_poly(np.poly1d(xy[0]), x), self.__move_poly(np.poly1d(xy[1]), y)

    def __transform_poly(self, x, y, h, polyx_1, polyy_1, polyx_2, polyy_2):
        h = h - np.pi/2
        polyx_1, polyy_1 = self.__rotate_poly(polyx_1, polyy_1, h)
        polyx_2, polyy_2 = self.__rotate_poly(polyx_2, polyy_2, h)
        polyx_1 = self.__move_poly(polyx_1, x)
        polyy_1 = self.__move_poly(polyy_1, y)
        polyx_2 = self.__move_poly(polyx_2, x)
        polyy_2 = self.__move_poly(polyy_2, y)
        return polyx_1, polyy_1, polyx_2, polyy_2

    # heading in rad
    def get_pull_out_left_polys(self, x, y, heading):
        polyx_curve, polyy_curve, polyx_straight, polyy_straight = self.__transform_poly(x, y, heading, *self.gen.gePullOutLeftTrajectoryPolynomials())
        return polyx_straight, polyy_straight, self.__invert_poly(polyx_curve), self.__invert_poly(polyy_curve)

    def get_pull_out_right_polys(self, x, y, heading):
        polyx_curve, polyy_curve, polyx_straight, polyy_straight = self.__transform_poly(x, y, heading, *self.gen.getParkingTrajectoryPolynomials())
        return polyx_straight, polyy_straight, polyx_curve, polyy_curve

    def get_parking_left_poly(self, x, y, heading):
        polyx_curve, polyy_curve, polyx_straight, polyy_straight = self.gen.gePullOutLeftTrajectoryPolynomials()
        polyx_curve, polyy_curve, polyx_straight, polyy_straight = self.__transform_poly(x, y, heading, -polyx_curve, polyy_curve, -polyx_straight, polyy_straight)
        return polyx_curve, polyy_curve, self.__invert_poly(polyx_straight), self.__invert_poly(polyy_straight)

    def get_parking_right_poly(self, x, y, heading):
        polyx_curve, polyy_curve, polyx_straight, polyy_straight = self.gen.getParkingTrajectoryPolynomials()
        polyx_curve, polyy_curve, polyx_straight, polyy_straight = self.__transform_poly(x, y, heading, polyx_curve, -polyy_curve, polyx_straight, -polyy_straight)
        return self.__invert_poly(polyx_curve), self.__invert_poly(polyy_curve), self.__invert_poly(polyx_straight), self.__invert_poly(polyy_straight)

def get_rot_mat(th):
    return np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])

if __name__ == '__main__':
    parkingPlanner = ParkingPlanner()
    a, b, c, d = parkingPlanner.get_pull_out_left_polys(0, 0, 0)
    plt.plot(a(np.linspace(0, 0.8, 100)), b(np.linspace(0, 0.8, 100)), 'r.')
    plt.plot(a(np.linspace(0.8, 1, 100)), b(np.linspace(0.8, 1, 100)), 'k.')
    plt.plot(c(np.linspace(0, 0.8, 100)), d(np.linspace(0, 0.8, 100)), 'b.')
    plt.plot(c(np.linspace(0.8, 1, 100)), d(np.linspace(0.8, 1, 100)), 'g.')
    plt.show()

'''steering_angle = np.zeros( ParkingTrajectoryGenerator1._ParkingTrajectoryGenerator__parkingTrajectoryPoints_y_rear_axle.size)
    i = 0
    for elem in ParkingTrajectoryGenerator1._ParkingTrajectoryGenerator__parkingTrajectoryPoints_y_rear_axle:
        steering_angle[i] = ParkingTrajectoryGenerator1.getSteeringAngle(
            ParkingTrajectoryGenerator1._ParkingTrajectoryGenerator__parkingTrajectoryPoints_y_rear_axle[i],
            ParkingTrajectoryGenerator1._ParkingTrajectoryGenerator__theta[i])
        i += 1

    plt.stem(ParkingTrajectoryGenerator1._ParkingTrajectoryGenerator__theta)
    plt.show()

    plt.stem(np.degrees(steering_angle))
    plt.show()

    # ParkingTrajectoryGenerator1.calcPullOutLeftTrajectory()'''
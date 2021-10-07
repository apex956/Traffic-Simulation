# -*- coding: utf-8 -*-
"""
Created on Tue Aug 29 17:57:05 2017
"""

from enum import Enum, unique
import random
import pygame
import numpy as np
from constant import Color, Const


class TrafficSimulation:
    """      This is the main class for running the simulation     """

    list_of_roads = []
    speed_factor = 40  # slow down the simulation by increasing this factor

    @classmethod
    def initialize_simulation(cls):
        # create a single car factory
        cls.car_factory = CarFactory()

        # create roads
        # length and width of roads and lanes are in meters
        # Speed is in km/hr
        length = Const.TOTAL_LENGTH_OF_5_SEG_ROAD

        road = Road(label="Main Highway", length=length,
                    no_of_lanes=3, lane_width=4,
                    directionality="uni", exit_loc=[length],
                    entry_loc=[0], speed_spec=Const.USED_SPEED_SPEC)
        cls.list_of_roads.append(road)  # add 1st road to list

        # create a control center
        control_center = ControlCenter()
        control_center.run_control_center()


class Road:
    """ ---- class Road ---- """

    def __init__(self, **args):
        self.label = args.get("label")
        self.length = args.get("length")
        self.no_of_lanes = args.get("no_of_lanes")
        self.lane_width = args.get("lane_width")
        self.road_width = self.no_of_lanes * self.lane_width
        # create lanes
        self.list_of_lanes = []
        self.speed_spec = args.get("speed_spec")
        for lane_idx in range(self.no_of_lanes):
            self.list_of_lanes.append(Lane(lane_idx, 0))
        self.disturb_500 = False   # used to simulate disturbances on the road


class Lane:
    """ ---- class Lane ---- """
    
    def __init__(self, rl_loc, direction):
        self.rl_loc = rl_loc
        self.direction = direction
        self.color = Color.BLACK
               
        self.list_of_vehicles = {}  # a dictionary with car ID as key
        self.last_vehicle = None  # the last vehicle on the road
        self.first_vehicle = None  # the first vehicle on the road
        self.entry_dist = 30  # [m] basic distance of entering car from car ahead
        self.delay_index = 0  # used to slow the variability
        self.entry_var = 0  # variability of entry distance

    def move_vehicles(self, road):
        car_factory = TrafficSimulation.car_factory

        # calculate dynamic properties of vehicles in lane
        for cid in self.list_of_vehicles:
            car = self.list_of_vehicles[cid]
            car.calc_car_params()

        # add the 1st car
        if len(self.list_of_vehicles) == 0:
            car = car_factory.create_random_vehicle()
            loc = -car.length
            speed = 50  # km/hr
            cid_of_car_ahead = None
            car.set_car_on_road(road, self, loc, speed, cid_of_car_ahead)
            self.list_of_vehicles[car.cid] = car
            self.last_vehicle = car
            self.first_vehicle = car

        # Add a new vehicle
        self.delay_index += 1
        if self.delay_index % 60 == 0:
            self.entry_var = random.randrange(int(-self.entry_dist/2),
                                              int(self.entry_dist/2))
        if self.last_vehicle.loc >= (self.entry_dist + self.entry_var):
            car = car_factory.create_random_vehicle()
            loc = -car.length
            speed = self.last_vehicle.speed  # km/hr
            cid_of_car_ahead = self.last_vehicle.cid
            car.set_car_on_road(road, self, loc, speed, cid_of_car_ahead)
            # update the car ahead about cid of following car
            car_ahead = self.list_of_vehicles[cid_of_car_ahead]
            car_ahead.cid_of_car_behind = car.cid
            # add car to list of vehicles on this lane
            self.list_of_vehicles[car.cid] = car
            self.last_vehicle = car

        # drop cars at end of road
        car = self.first_vehicle
        if (car.loc + car.length) >= road.length:
            # update the car behind
            #print("DEBUG: ID, Speed, speed limit factor: ",
            #      car.cid, car.speed, car.speed_limit_factor)
            car_behind = self.list_of_vehicles[car.cid_of_car_behind]
            car_behind.cid_of_car_ahead = None
            car_behind.loc_of_car_ahead = None
            # assign new 1st car for this lane
            self.first_vehicle = car_behind
            # delete from dictionary for lane
            del self.list_of_vehicles[car.cid]


class Car(object):
    """----   class Car ---- """

    def __init__(self, cid, color, length, width, policy):
        self.width = width  # [m]
        self.cid = cid
        self.color = color
        self.length = length
        self.policy = policy
        self.delay_index = 0
        self.dbg_delay_index = 0
        self.cid_of_car_behind = None
        self.dist_factor = 1
        self.speed_limit_factor = 1
        self.start_timer = 0

    def set_car_on_road(self, road, lane, loc, speed, cid_of_car_ahead):
        self.road = road
        self.lane = lane
        self.loc = loc
        self.speed = speed  # [km/hr]
        self.cid_of_car_ahead = cid_of_car_ahead
        if cid_of_car_ahead is None:
            self.loc_of_car_ahead = None
            self.speed_of_car_ahead = 150
        self.breaks = False

    def calc_car_params(self):
        """
        Calculate car new location and speed
        Speed is calculated several ways and overrides previous calculations
        For example, speed limit overrides large distance from car ahead
        """
        self.breaks = False
        speed = self.speed

        if self.cid_of_car_ahead is not None:
            car_ahead = self.lane.list_of_vehicles[self.cid_of_car_ahead]
            self.loc_of_car_ahead = car_ahead.loc
            self.speed_of_car_ahead = car_ahead.speed

        if self.delay_index % 400 == 0:
            self.dist_factor = abs(1 + np.random.normal(0, 0.3))
            if self.dist_factor < 0.2:
                self.dist_factor = 0.2
            self.speed_limit_factor = abs(1 + np.random.normal(0, 0.3))
        self.delay_index += 1

        # distance from car ahead
        if self.loc_of_car_ahead is not None:
            actual_dist = self.loc_of_car_ahead - (self.loc + self.length)
        else:
            actual_dist = self.road.length  # unlimited

        # DEBUG
        if self.cid == 1000000 and self.dbg_delay_index % 100 == 0:
            print("------------------------")
            print("DEBUG: Car cid : ", self.cid)
            print("DEBUG: Car loc : ", self.loc)
            print("DEBUG: speed: ", self.speed)
        self.dbg_delay_index += 1

        # The desired minimum distance from the car ahead based on speed
        speed_m_p_s = self.speed / 3.6
        if self.policy == 0:
            min_dist_base = speed_m_p_s * 1.0   # in [m]
            deceleration = 5  # [km/hr per second]
            acceleration = 2.5  # [km/hr per second]
        elif self.policy == 1:
            min_dist_base = speed_m_p_s * 0.8   # in [m]
            deceleration = 4.0  # [km/hr per second]
            acceleration = 2.0  # [km/hr per second]
        else:
            min_dist_base = speed_m_p_s * 1.2   # in [m]
            deceleration = 3.0  # [km/hr per second]
            acceleration = 1.5  # [km/hr per second]

        min_dist = min_dist_base * self.dist_factor
        acceleration_factor = TrafficSimulation.speed_factor / 8
        acceleration /= acceleration_factor
        deceleration /= acceleration_factor
        mild_deceleration = deceleration / 2

        # maintain a reasonable distance
        if actual_dist > min_dist * 2:
            if self.speed <= self.speed_of_car_ahead:
                speed = self.speed + acceleration  # assuming calc every second

        # slow down ahead of time
        if actual_dist < min_dist * 1.5:
            if self.speed > self.speed_of_car_ahead:
                speed = self.speed - mild_deceleration

        # impose speed limits
        for spec in self.road.speed_spec:
            from_loc = spec[0]
            to_loc = spec[1]
            limit = spec[2]
            if self.lane.rl_loc == self.road.no_of_lanes-1:
                limit *= 0.7  # the right lane is slower
            if self.loc > from_loc and self.loc < to_loc:
                if self.speed * self.speed_limit_factor > limit:
                    speed = self.speed - mild_deceleration

        # periodic disturbance
        # need to move most of this code to the road class
        disturb = False
        self.road.disturb_500 = False
        length_of_disturbance = 20  # seconds
        period_of_disturbance = 120  # seconds
        time = int(pygame.time.get_ticks()/1000)
        if time % period_of_disturbance == 0:
            self.start_timer = time
        if time < (self.start_timer + length_of_disturbance):
            disturb = True
        if disturb is True:
            self.road.disturb_500 = True
#            if self.dbg_delay_index % 700 == 0:
#                print("Traffic disturbance at 500 m:", time)
            # location of disturbance
            from_loc = 500
            to_loc = 600
            limit = 40 # reduced speed limit
            if self.loc > from_loc and self.loc < to_loc:
                if self.speed * self.speed_limit_factor > limit:
                    speed = self.speed - deceleration
                    self.breaks = True

        # maintain a minimum distance from car ahead
        if actual_dist < min_dist:
            if self.speed > self.speed_of_car_ahead:
                speed = self.speed - deceleration
                self.breaks = True

        # avoid collision
        if self.speed >= self.speed_of_car_ahead:
            if actual_dist < 3 and self.speed > 20:
                speed = self.speed_of_car_ahead/2
                self.breaks = True
                print("++++++ DEBUG: Impending collision. Lane: ",
                      self.lane.rl_loc, "Speed: ", self.speed, "Location: ", self.loc)
            elif actual_dist < min_dist_base / 2:
                print("++++++ DEBUG: Too close to car ahead. Lane: ",
                      self.lane.rl_loc, "Location: ", self.loc)
                speed = self.speed_of_car_ahead * 3/4
                self.breaks = True

        # DEBUG
        """
        if self.cid == 10 and self.dbg_delay_index % 100 == 0:
            print("------------------------")
            print("DEBUG: speed for cid 10: ", speed_m_p_s)
            print("DEBUG: cid of car ahead: ", self.cid_of_car_ahead)
            print("DEBUG: min_dist: ", min_dist)
            #print("DEBUG: act_dist: ", actual_dist)
            #print("DEBUG: loc of car ahead: ", self.loc_of_car_ahead)
        self.dbg_delay_index += 1
        """

        # periodic calc may be much faster than once a second
        self.speed = speed
        speed_m_p_s = speed / 3.6
        speed_m_p_s /= TrafficSimulation.speed_factor
        self.loc += speed_m_p_s


class CarFactory():
    """  ---- class CarFactory ----
    There are three typeS of cars: bus/truck, regular, small.
    only a small proportion of vehicles are buses or trucks
    length and width are in meters
    """
    cid = 0
    bus_truck_choice = 12 # one vehicle out of this number is a truck or a bus

    def __init__(self):
        pass

    def create_random_vehicle(self):
        """random color, length, behavior (policy)"""
        CarFactory.cid += 1
        cid = CarFactory.cid
        # a bus or truck
        if random.choice(range(CarFactory.bus_truck_choice)) == 1:
            length = random.choice([9, 10, 11])
            width = 3
            color = random.choice([Color.RED, Color.GREEN,
                               Color.YELLOW, Color.WHITE,
                               Color.SGI_GRAY_52,  Color.SILVER])
        else:  # regular car
            length = random.choice([4.5, 5.0, 5.5, 6.0])
            width = random.choice([2.2, 2.5])
            color = random.choice([Color.BLUE, Color.GREEN,
                                   Color.YELLOW, Color.WHITE,
                                   Color.RED, Color.CRIMSON,
                                   Color.MAGENTA, Color.SGI_GRAY_52,
                                   Color.ORANGE, Color.SILVER,
                                   Color.TURQUOISE])
            # a small car
            if length <= 4.5:
                width = 2

        policy = random.choice(range(3))
        car = Car(cid, color, length, width, policy)
        return car


class ControlCenter():
    """---- class ControlCenter ----
    Each road segment is 300 meters
    """

    def __init__(self):
        self.NO_OF_ROAD_SEGMENTS = 5
        self.road_seg_gap = 40    # pixels
        self.intra_road_gap = 100    # pixels
        self.line_width = 1  # pixels
        self.line_color = Color.WHITE

    @classmethod
    def m_to_p(cls, meters):
        """Meters to pixels"""
        return meters * 6

    def run_control_center(self):
        pygame.init()
        screen_size = (1840, 800)  # pixels
        screen_width = screen_size[0]
        x_margin = 20   # pixels
        top_margin = 80  # pixels
        screen = pygame.display.set_mode(screen_size)
        pygame.display.set_caption("Simple Traffic Simulation")
        # Loop until the user clicks the close button.
        done = False

        # sound effects
        pygame.mixer.music.load('traffic-13.mp3')
        pygame.mixer.music.play(-1)

        # Used to manage how fast the screen updates
        clock = pygame.time.Clock()

        # font, size, bold, italics
        font = pygame.font.SysFont('Calibri', 25, True, False)
        # Render the text. "True" means anti-aliased text.
        text = font.render("Simple Traffic Simulation", True,
                           Color.BLACK)

        # define starting point of each road segment
        road_start_x = []
        road_start_y = []
        for road_idx, road in enumerate(TrafficSimulation.list_of_roads):
            road_width_p = ControlCenter.m_to_p(road.road_width)
            road_seg_start_x = []
            road_seg_start_y = []
            for road_seg_idx in range(self.NO_OF_ROAD_SEGMENTS):
                road_seg_start_x.append(x_margin)
                road_seg_start_y.append(top_margin
                                        + (self.road_seg_gap + road_width_p)
                                        * road_seg_idx
                                        + self.intra_road_gap * road_idx)
            #  For each road the starting point of the road segments    
            road_start_x.append(road_seg_start_x)
            road_start_y.append(road_seg_start_y)

        # -------- Main Loop -----------
        while not done:
            # --- Main event loop
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            screen.fill(Color.FOREST_GREEN)

            # --- Drawing code should go here
            # Put the text at center of screen
            screen.blit(text, [(screen_width - 300)/2, 10])

            road_seg_length = screen_width - 2 * x_margin

            for road_idx, road in enumerate(TrafficSimulation.list_of_roads):
                lane_width_p = ControlCenter.m_to_p(road.lane_width)
                for lane_idx, lane in enumerate(road.list_of_lanes):
                    for road_seg_idx in range(self.NO_OF_ROAD_SEGMENTS):
                        lane_start_x = road_start_x[road_idx][road_seg_idx]
                        lane_start_y = (road_start_y[road_idx][road_seg_idx]
                                        + lane_width_p * lane_idx)
                        #  Draw lanes
                        pygame.draw.rect(screen, lane.color,
                                         [lane_start_x, lane_start_y,
                                          road_seg_length, lane_width_p])
                        # Draw lane separation lines
                        line_start_x = lane_start_x
                        line_start_y = lane_start_y
                        line_end_x = line_start_x + road_seg_length
                        line_end_y = line_start_y
                        pygame.draw.line(screen, self.line_color, 
                                         [line_start_x, line_start_y],
                                         [line_end_x, line_end_y], 
                                         self.line_width)
                        # Draw road segments lines
                        line_start_x = road_start_x[road_idx][road_seg_idx]
                        line_start_y = road_start_y[road_idx][road_seg_idx]
                        line_end_x = line_start_x + road_seg_length
                        line_end_y = line_start_y
                        pygame.draw.line(screen, self.line_color, 
                                         [line_start_x, line_start_y],
                                         [line_end_x, line_end_y],
                                         self.line_width)
                        line_start_y += ControlCenter.m_to_p(road.road_width)
                        line_end_y = line_start_y
                        pygame.draw.line(screen, self.line_color,
                                         [line_start_x, line_start_y],
                                         [line_end_x, line_end_y],
                                         self.line_width)

                        #draw milestones
                        txt_color = Color.WHITE
                        font = pygame.font.SysFont('Calibri', 18, True, False)
                        ms_width = self.line_width+2
                        ms_height = 10
                        line_start_x = road_start_x[road_idx][road_seg_idx]
                        line_start_y = road_start_y[road_idx][road_seg_idx]
                        line_end_x = line_start_x 
                        line_end_y = line_start_y - ms_height
                        pygame.draw.line(screen, self.line_color, 
                                         [line_start_x, line_start_y],
                                         [line_end_x, line_end_y],
                                         ms_width)
                        # text offset
                        x_offset = 5
                        y_offset = -23
                        txt1 = str(road_seg_idx*300)
                        text1 = font.render(txt1+"[m]", True, txt_color)
                        screen.blit(text1, [line_start_x + x_offset,
                                            line_start_y + y_offset])

                        line_start_x = (road_start_x[road_idx][road_seg_idx]
                                        + road_seg_length - ms_width)
                        line_start_y = road_start_y[road_idx][road_seg_idx]
                        line_end_x = line_start_x
                        line_end_y = line_start_y - ms_height
                        pygame.draw.line(screen, self.line_color,
                                         [line_start_x, line_start_y],
                                         [line_end_x, line_end_y],
                                         ms_width)

                        # text offset
                        x_offset = -60
                        y_offset = -23
                        txt2 = str((1+road_seg_idx)*300)
                        text2 = font.render(txt2+"[m]", True, txt_color)
                        screen.blit(text2, [line_start_x + x_offset,
                                            line_start_y + y_offset])
                
                        line_start_x = (road_start_x[road_idx][road_seg_idx]
                                        + road_seg_length/2 - ms_width)
                        line_start_y = road_start_y[road_idx][road_seg_idx]
                        line_end_x = line_start_x
                        line_end_y = line_start_y - ms_height
                        pygame.draw.line(screen, self.line_color,
                                         [line_start_x, line_start_y],
                                         [line_end_x, line_end_y],
                                         ms_width)

                        # text offset
                        x_offset = 5
                        y_offset = -23
                        txt1 = str(road_seg_idx*300 + 150)
                        text1 = font.render(txt1+"[m]", True, txt_color)
                        screen.blit(text1, [line_start_x + x_offset,
                                            line_start_y + y_offset])

                    # Disturbance on the road at 500 m milestone
                    if road.disturb_500 is True:
                        x_loc = 200/300*1800 + x_margin
                        y_loc = 185
                        base = 30
                        height = 20
                        pygame.draw.polygon(screen, Color.RED,
                                            [[x_loc, y_loc], 
                                             [x_loc + base, y_loc], 
                                             [x_loc + (base/2), y_loc-height]], 0)

                    # add/remove/move cars
                    lane.move_vehicles(road)

                    # Draw cars
                    for cid in lane.list_of_vehicles:
                        car = lane.list_of_vehicles[cid]
                        car_color = car.color
                        car_length = ControlCenter.m_to_p(car.length)
                        car_width = ControlCenter.m_to_p(car.width)

                        car_loc_p = ControlCenter.m_to_p(car.loc)
                        
                        # car entering the road
                        if car_loc_p <  0:
                            car_start_x = x_margin
                            car_length += car_loc_p
                            road_seg_idx = 0
                            car_at_end = False
                        # car on the road    
                        else:
                            # car location along the road
                            road_seg_idx = int(car_loc_p // road_seg_length)
                            if road_seg_idx >= self.NO_OF_ROAD_SEGMENTS:
                                print("DEBUG: road seg index:", road_seg_idx)
                                done = True
                                break
                            x_seg_loc = car_loc_p - (road_seg_idx * road_seg_length)
                            car_start_x = x_margin + x_seg_loc

                            # car at end of segment
                            car_at_end = False
                            if (x_seg_loc + car_length) > road_seg_length:
                                car_length = road_seg_length - x_seg_loc
                                car_at_end = True

                        lane_start_y = (road_start_y[road_idx][road_seg_idx]
                                        + lane_width_p * lane_idx)
                        car_start_y = (lane_start_y + lane_width_p/2
                                       - car_width/2)
                        pygame.draw.rect(screen, car_color,
                                         [car_start_x, car_start_y,
                                          car_length, car_width])
                        # Breaks
                        break_x_p = 4
                        break_y_p = 4
                        break_x_ext_p = 2
                        if car.breaks is True:
                            pygame.draw.rect(screen, Color.RED,
                                             [car_start_x-break_x_ext_p,
                                              car_start_y,
                                              break_x_p, break_y_p])
                            pygame.draw.rect(screen, Color.RED,
                                             [car_start_x-break_x_ext_p,
                                             car_start_y+car_width-break_y_p,
                                             break_x_p, break_y_p])

                        # car entering a segment
                        if car_at_end is True:  # at end of previous segment
                            car_length = ControlCenter.m_to_p(car.length) - car_length
                            car_start_x = x_margin
                            lane_start_y = (road_start_y[road_idx][road_seg_idx + 1]
                                            + lane_width_p * lane_idx)
                            car_start_y = (lane_start_y + lane_width_p/2
                                           - car_width/2)    
                            pygame.draw.rect(screen, car_color,
                                             [car_start_x, car_start_y,
                                              car_length, car_width])

            # update the screen 
            pygame.display.flip()

            # --- Limit to 60 frames per second
            clock.tick(60) 

        # Close the window and quit.
        pygame.quit()


#==============================================================================
#     Run the simulation
#==============================================================================
TrafficSimulation.initialize_simulation()


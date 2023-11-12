import pygame
from autonavsim2d.utils.utils import generate_path, generate_path_custom, generate_waypoints_v4, compare_waypoints, parse_arrow_angle, RED, BLACK, WHITE, GREEN, GREY, ORANGE, BLUE, RED_LIGHT, GREY_LIGHT
from autonavsim2d.utils.robot_model import Robot
from autonavsim2d.utils.logger import Logger
import math
import pkg_resources



class AutoNavSim2D:

    # initialise pygame font
    pygame.font.init()

    # dev custom params
    planner = None
    custom_planner = None
    main_window_select = True
    path_planning_window_select = False

    # window params
    WIN_WIDTH, WIN_HEIGHT = 1147, 872
    WIN_WIDTH_FULL, WIN_HEIGHT_FULL = 1447, 872

    # active window
    ACTIVE_WINDOW = None

    # main window
    WIN = pygame.display.set_mode((WIN_WIDTH_FULL, WIN_HEIGHT_FULL))

    # path planning window
    WIN_PATH_PLANNING = pygame.display.set_mode((WIN_WIDTH_FULL, WIN_HEIGHT_FULL))

    # set window name
    pygame.display.set_caption("AutoNavSim")

    # fonts
    START_FONT = pygame.font.SysFont('comicsans', 30)
    NAVIGATE_FONT = pygame.font.SysFont('comicsans', 30)
    RESET_FONT = pygame.font.SysFont('comicsans', 30)
    ALGORITHM_NAME_FONT = pygame.font.SysFont('comicsans', 30)
    TIME_TAKEN_FONT = pygame.font.SysFont('comicsans', 25)
    METRICS_FONT = pygame.font.SysFont('comicsans', 25)
    WAYPOINTS_FONT = pygame.font.SysFont('comicsans', 25)
    LOG_MSGS_FONT = pygame.font.SysFont('comicsans', 20)

    TITLE_FONT = pygame.font.SysFont('comicsans', 60)
    TITLE_DESCRIPTION_FONT = pygame.font.SysFont('comicsans', 30)
    MAIN_BUTTON_FONT = pygame.font.SysFont('comicsans', 27)
    MAIN_SMALL_TEXT = pygame.font.SysFont('comicsans', 25)

    # images
    HEADING_IMAGE = pygame.transform.scale(pygame.image.load(pkg_resources.resource_filename('autonavsim2d', 'utils/assets/dashboard_heading.png')), (100, 100))
    # HEADING_IMAGE = pygame.transform.scale(pygame.image.load('src/autonavsim2d/assets/dashboard_heading.png'), (100, 100))

    ARROW_IMAGE = pygame.transform.scale(pygame.image.load(pkg_resources.resource_filename('autonavsim2d', 'utils/assets/heading_arrow.png')), (20, 50))

    # main window images
    LOGO_IMAGE = pygame.transform.scale(pygame.image.load(pkg_resources.resource_filename('autonavsim2d', 'utils/assets/logo.png')), (150, 150))
    BACKGROUND_IMAGE = pygame.transform.scale(pygame.image.load(pkg_resources.resource_filename('autonavsim2d', 'utils/assets/background2.jpg')), (WIN_WIDTH_FULL, WIN_HEIGHT_FULL))

    def __init__(self, custom_planner=None, window=None):
        # set window
        if window == 'default':
            self.ACTIVE_WINDOW = self.WIN
            self.main_window_select = True
            self.path_planning_window_select = False

        elif window == 'amr':
            self.ACTIVE_WINDOW = self.WIN_PATH_PLANNING
            self.main_window_select = False
            self.path_planning_window_select = True
        
        else:
            self.ACTIVE_WINDOW = self.WIN
            self.main_window_select = True
            self.path_planning_window_select = False

        # set path planner
        if custom_planner == 'default':
            # set planner to default
            self.planner = 'default'

        else:
            # set dev custom planner
            self.custom_planner = custom_planner


    def generate_grid(self):
        # empty grid
        grid = []

        # loop indices for matrix positions
        x, y  = 0, 0

        # generate 2x2 matrix for rows and cols
        for i in range(0, self.WIN_HEIGHT, 23):
            rows = []
            y = 0
            for j in range (0, self.WIN_WIDTH, 23):
                cell = [pygame.rect.Rect(j, i, 20, 20), GREY, (x, y)]
                rows.append(cell)
                y += 1

            grid.append(rows)
            x += 1

        return grid


    def draw_robot_frame(self, ACTIVE_WINDOW, center, rotation):
        n=50
        center_x, center_y = center
        x_axis = (center_x + n*math.cos(-rotation), center_y + n*math.sin(-rotation))
        y_axis = (center_x + n*math.cos(-rotation + math.pi/2), center_y + n*math.sin(-rotation + math.pi/2))

        # draw lines
        pygame.draw.line(ACTIVE_WINDOW, GREEN, (center_x, center_y), x_axis, 3)
        pygame.draw.line(ACTIVE_WINDOW, RED_LIGHT, (center_x, center_y), y_axis, 3)
        

    def generate_grid_matrix(self, grid):
        GRID_LEN = len(grid)
        GRID_WIDTH = len(grid[0])
        grid_matrix = []

        for i in range(GRID_LEN):
            row_matrix = []
            for j in range(GRID_WIDTH):
                cell = grid[i][j]

                # populate grid matrix
                if cell[1] == GREY or cell[1] == RED or cell[1] == GREEN or cell[1] == BLUE or cell[1] == ORANGE:    # free space
                    row_matrix.append(1)
                else: 
                    row_matrix.append(0)

            # append row to matrix
            grid_matrix.append(row_matrix)
        
        return grid_matrix


    def draw_grid(self, ACTIVE_WINDOW, grid):
        # horizontal separator dist
        separator_dist = 0

        # draw cells
        for i in range(len(grid)):
            row = grid[i]
            for rect in row:
                pygame.draw.rect(ACTIVE_WINDOW, rect[1], rect[0])


    def draw_start_button(self, ACTIVE_WINDOW, button, color):
        pygame.draw.rect(ACTIVE_WINDOW, color, button, border_radius=5)

        # add text to button
        text = self.START_FONT.render("Plan Path", 1, BLACK)
        ACTIVE_WINDOW.blit(text, (self.WIN_WIDTH+104, 405))


    def draw_navigate_button(self, ACTIVE_WINDOW, button, color):
        pygame.draw.rect(ACTIVE_WINDOW, color, button, border_radius=5)

        # add text to button
        text = self.NAVIGATE_FONT.render("Navigate", 1, WHITE)
        ACTIVE_WINDOW.blit(text, (self.WIN_WIDTH+110, 465))


    def draw_reset_button(self, ACTIVE_WINDOW, button, color):
        pygame.draw.rect(ACTIVE_WINDOW, color, button, border_radius=5)

        # add text to button
        text = self.RESET_FONT.render("Reset", 1, WHITE)
        ACTIVE_WINDOW.blit(text, (self.WIN_WIDTH+125, 525))


    def draw_path(self, generated_path, grid, start, end, color, draw_new_path):
        # draw new map
        for loc in generated_path:
            node = grid[loc[0]][loc[1]]
            
            if node == start or node ==  end:
                pass

            else:
                # set color to black
                node[1] = color

        pygame.display.update()


    def clear_map_path(self, generated_path, grid, start, end, color):
        # reset grid 
        for loc in generated_path:
            node = grid[loc[0]][loc[1]]
            
            if node == start or node ==  end:
                pass

            else:
                # set color to GREY
                node[1] = GREY

        pygame.display.update()


    def clear_map_path_all(self, generated_path, grid, start, end, color):
        x, y  = 0, 0

        for i in range(0, self.WIN_HEIGHT, 23):
            y = 0
            for j in range (0, self.WIN_WIDTH, 23):
                node = grid[x][y]
                node[1] = GREY
                y += 1
            
            x += 1

        pygame.display.update()


    def draw_spline_path(self, spline_path):
        for i in range(len(spline_path)):
            rect = pygame.rect.Rect(spline_path[i][0], spline_path[i][1], 20, 20)
            pygame.draw.rect(self.WIN_PATH_PLANNING, ORANGE, rect)
            

    def draw_text(self, ACTIVE_WINDOW, time_taken):
        # algorithm name text
        name_text = self.ALGORITHM_NAME_FONT.render("Algorithm: Dijkstra", 1, BLACK)
        ACTIVE_WINDOW.blit(name_text, (self.WIN_WIDTH+55, 30))

        # time taken text
        time_text = self.TIME_TAKEN_FONT.render('Time Taken: ' + str(round(time_taken*1000, 2)) + "ms", 1, BLACK)
        ACTIVE_WINDOW.blit(time_text, (self.WIN_WIDTH+ 75, 70))


    def draw_dashboard(self, ACTIVE_WINDOW, angle, x, y, vl, vr, counter, waypoints_len, logger):
        # heading compass image
        heading_rect = pygame.rect.Rect(self.WIN_WIDTH+5, 140, 100, 100)
        ACTIVE_WINDOW.blit(self.HEADING_IMAGE, heading_rect)

        # # heading arrow image
        if angle is not None: 
            ARROW_IMAGE_CPY = pygame.transform.rotate(self.ARROW_IMAGE, math.degrees(parse_arrow_angle(angle)))
            ARROW_IMAGE_CPY_RECT = ARROW_IMAGE_CPY.get_rect(center=heading_rect.center)
            ACTIVE_WINDOW.blit(ARROW_IMAGE_CPY, ARROW_IMAGE_CPY_RECT)

        # differential drive metrics
        vl_text = self.METRICS_FONT.render('vl: ' + str(round(vl, 1)), 1, BLACK)
        vr_text = self.METRICS_FONT.render('vr: ' + str(round(vr, 1)), 1, BLACK)
        if angle is None:
            theta_text = self.METRICS_FONT.render('θ: 0', 1, BLACK)
        else:
            theta_text = self.METRICS_FONT.render('θ: ' + str(math.degrees(angle)), 1, BLACK)

        ACTIVE_WINDOW.blit(vl_text, (self.WIN_WIDTH+105, 150))
        ACTIVE_WINDOW.blit(vr_text, (self.WIN_WIDTH+165, 150))
        ACTIVE_WINDOW.blit(theta_text, (self.WIN_WIDTH+230, 150))

        # divider
        pygame.draw.line(ACTIVE_WINDOW, GREY, (self.WIN_WIDTH+122, 180), (self.WIN_WIDTH+270, 180), width=1)

        # robot pose
        x_text = self.METRICS_FONT.render('x: ' + str(round(x, 2)), 1, BLACK)
        y_text = self.METRICS_FONT.render('y: ' + str(round(y, 2)), 1, BLACK)
        z_text = self.METRICS_FONT.render('z: 0', 1, BLACK)

        ACTIVE_WINDOW.blit(x_text, (self.WIN_WIDTH+107, 190))
        ACTIVE_WINDOW.blit(y_text, (self.WIN_WIDTH+185, 190))
        ACTIVE_WINDOW.blit(z_text, (self.WIN_WIDTH+265, 190))

        # divider
        pygame.draw.line(ACTIVE_WINDOW, GREY, (self.WIN_WIDTH+122, 220), (self.WIN_WIDTH+270, 220), width=1)

        # waypoints completed
        waypoint_text = self.WAYPOINTS_FONT.render(f'Waypoints: {counter} of {waypoints_len}', 1, BLACK)
        ACTIVE_WINDOW.blit(waypoint_text, (self.WIN_WIDTH+110, 230))

        # logs title
        logs_title_text = self.WAYPOINTS_FONT.render('Logs', 1, BLACK)
        ACTIVE_WINDOW.blit(logs_title_text, (self.WIN_WIDTH+10, 260))

        # logs field
        logs_field_rect = pygame.rect.Rect(self.WIN_WIDTH+10, 290, 280, 90)
        pygame.draw.rect(ACTIVE_WINDOW, GREY_LIGHT, logs_field_rect)

        # log messages
        msgs = logger.get_logs()
        adder = 0
        
        for i in range(len(msgs)):
            log_msgs_text = self.LOG_MSGS_FONT.render(msgs[i], 1, BLACK)
            ACTIVE_WINDOW.blit(log_msgs_text, (self.WIN_WIDTH+15, 300+adder))
            adder += 15


    def draw_diff_drive_button(self, diff_drive_btn):
        pygame.draw.rect(self.WIN, GREEN, diff_drive_btn, border_radius=10)

        # text
        diff_drive_text = self.MAIN_BUTTON_FONT.render("Autonomous Mobile Robot Simulation - Differential Drive", 1, BLACK)
        self.WIN.blit(diff_drive_text, (self.WIN_WIDTH_FULL/2 - diff_drive_text.get_width()/2, 375))


    def draw_skid_steer_button(self, skid_steer_button):
        pygame.draw.rect(self.WIN, RED, skid_steer_button, border_radius=10)

        # text
        diff_drive_text = self.MAIN_BUTTON_FONT.render("Autonomous Mobile Robot Simulation - Skid-Steer Drive", 1, WHITE)
        self.WIN.blit(diff_drive_text, (self.WIN_WIDTH_FULL/2 - diff_drive_text.get_width()/2, 475))


    def draw_self_driving_vehicle_button(self, self_driving_vehicle_button):
        pygame.draw.rect(self.WIN, BLUE, self_driving_vehicle_button, border_radius=10)

        # text
        diff_drive_text = self.MAIN_BUTTON_FONT.render("4-Wheel Self-driving Vehicle", 1, WHITE)
        self.WIN.blit(diff_drive_text, (self.WIN_WIDTH_FULL/2 - diff_drive_text.get_width()/2, 575))


    def draw_path_planning_window(self, ACTIVE_WINDOW, grid, start_btn, start_btn_color, reset_btn, reset_btn_color, time_taken, navigate_button, navigate_btn_color, robot, dt, 
                    draw_frame, robot_angle_, counter, waypoints_len, logger, spline_path):
        # fill background with white color
        ACTIVE_WINDOW.fill(WHITE)

        self.draw_grid(ACTIVE_WINDOW, grid=grid)

        # draw buttons
        self.draw_start_button(ACTIVE_WINDOW, button=start_btn, color=start_btn_color)
        self.draw_navigate_button(ACTIVE_WINDOW, button=navigate_button, color=navigate_btn_color)
        self.draw_reset_button(ACTIVE_WINDOW, button=reset_btn, color=reset_btn_color)

        # draw robot on screen
        if robot is not None:
            robot.draw(ACTIVE_WINDOW)
        
        # draw frame
        if draw_frame and len(robot_angle_) == 0 and robot is not None:
            self.draw_robot_frame(ACTIVE_WINDOW, (robot.x, robot.y), robot_angle_[0])

        elif draw_frame and len(robot_angle_) != 0 and robot is not None:
            self.draw_robot_frame(ACTIVE_WINDOW, (robot.x, robot.y), robot_angle_[0])

        # display texts
        self.draw_text(ACTIVE_WINDOW, time_taken)

        # draw dashboard
        if robot is not None:
            self.draw_dashboard(ACTIVE_WINDOW, robot_angle_[0], robot.x, robot.y, vl=robot.vl, vr=robot.vl, counter=counter, waypoints_len=waypoints_len, logger=logger)

        else:
            self.draw_dashboard(ACTIVE_WINDOW, None, 0, 0, vl=0, vr=0, counter=0, waypoints_len=0, logger=logger)
        

        # update frame
        pygame.display.update()


    def draw_main_window(self, ACTIVE_WINDOW, diff_drive_btn, skid_steer_button, self_driving_vehicle_button):
        # fill background with white color
        ACTIVE_WINDOW.fill(WHITE)
        ACTIVE_WINDOW.blit(self.BACKGROUND_IMAGE, (0, 0))

        # logo
        ACTIVE_WINDOW.blit(self.LOGO_IMAGE, (self.WIN_WIDTH_FULL/2 - 320, 100))

        # title
        title_text = self.TITLE_FONT.render("AutoNav2D Simulator", 1, WHITE)
        ACTIVE_WINDOW.blit(title_text, (self.WIN_WIDTH_FULL/2 - 150, 130))

        # description
        description_text = self.TITLE_DESCRIPTION_FONT.render('Write code, Simulate Behaviour, Deploy to Innovation', 1, WHITE)
        ACTIVE_WINDOW.blit(description_text, (self.WIN_WIDTH_FULL/2 - 150, 180))

        # selection question
        selection_question = self.TITLE_DESCRIPTION_FONT.render('What would you like to do?', 1, WHITE)
        ACTIVE_WINDOW.blit(selection_question, (self.WIN_WIDTH_FULL/2 - selection_question.get_width()/2, 300))

        # buttons
        self.draw_diff_drive_button(diff_drive_btn)
        self.draw_skid_steer_button(skid_steer_button)
        self.draw_self_driving_vehicle_button(self_driving_vehicle_button)

        # version
        version_text = self.MAIN_SMALL_TEXT.render("v1.000", 1, WHITE)
        ACTIVE_WINDOW.blit(version_text, (10, self.WIN_HEIGHT_FULL - version_text.get_height() - 5))

        # open source text
        open_source_text = self.MAIN_SMALL_TEXT.render("AutoNavSim is OpenSouce on Github", 1, WHITE)
        ACTIVE_WINDOW.blit(open_source_text, (self.WIN_WIDTH_FULL/2  - open_source_text.get_width()/2, self.WIN_HEIGHT_FULL - open_source_text.get_height() - 5))

        # author
        author_text = self.MAIN_SMALL_TEXT.render("Author: Clinton Anani (twitter.com/oxncgen)", 1, WHITE)
        ACTIVE_WINDOW.blit(author_text, (self.WIN_WIDTH_FULL - author_text.get_width() - 10, self.WIN_HEIGHT_FULL - author_text.get_height() - 5))

        pygame.display.update()


    def run(self):

        # window selection params
        # self.ACTIVE_WINDOW = self.WIN

        home_button_clicked = False
        
        # loop params
        run = True
        FPS = 60
        clock = pygame.time.Clock()
        dt = 0
        lasttime = pygame.time.get_ticks()

        # logger
        logger = Logger()

        # generate grid
        grid = self.generate_grid()
        grid_cpy = grid

        # main window buttons
        diff_drive_btn = pygame.rect.Rect(self.WIN_WIDTH_FULL/2 - 300, 350, 600, 70)
        skid_steer_button = pygame.rect.Rect(self.WIN_WIDTH_FULL/2 - 300, 450, 600, 70)
        self_driving_vehicle_button = pygame.rect.Rect(self.WIN_WIDTH_FULL/2 - 300, 550, 600, 70)
        
        # path planning buttons
        start_btn = pygame.rect.Rect(self.WIN_WIDTH + 85, 390, 130, 50)
        start_btn_color = GREEN

        navigate_button = pygame.rect.Rect(self.WIN_WIDTH + 85, 450, 130, 50)
        navigate_btn_color = RED

        reset_btn = pygame.rect.Rect(self.WIN_WIDTH + 85, 510, 130, 50)
        reset_btn_color = BLUE

        # start and end locs
        start_coord = None
        end_coord = None

        # generated map
        path = []
        time_taken = 0
        matrix = None

        # navigation params
        m2p = 3779.52   # meters to pixels
        wheels_dist = 0.02117 * m2p

        # robot model
        robot = None
        robot_angle = math.pi/2
        robot_angle_ = []
        draw_frame = False

        # navigation animation params
        navigate = False
        navigate_2 = False
        draw_new_path = False
        robot_pose = None
        waypoints = []
        spline_path = []
        orientations_ep = []
        counter = 0
        VEL = 0.01
        WHEEL_DIST = 0.02117

        while run:
            clock.tick(FPS)

            # listen for events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run = False
                
                if pygame.mouse.get_pressed()[0]:   # left click
                    # get position of clicked
                    pos = pygame.mouse.get_pos()

                    # differential drive click
                    if self.main_window_select == True and diff_drive_btn.x <= pos[0] <=diff_drive_btn.x + diff_drive_btn.width and diff_drive_btn.y <= pos[1] <= diff_drive_btn.y + diff_drive_btn.height:
                        # change window state
                        if not home_button_clicked:
                            home_button_clicked = True
                            
                            self.main_window_select = False
                            self.path_planning_window_select = True
                            
                            # clear states
                            robot = None
                            start_coord = None
                            end_coord = None
                            
                            self.ACTIVE_WINDOW = self.WIN_PATH_PLANNING
                            home_button_clicked = False
                        
                    # skid steer drive click
                    if self.main_window_select == True and skid_steer_button.x <= pos[0] <=skid_steer_button.x + skid_steer_button.width and skid_steer_button.y <= pos[1] <= skid_steer_button.y + skid_steer_button.height:
                        pass

                    # self-driving vehicle click
                    if self.main_window_select == True and self_driving_vehicle_button.x <= pos[0] <=self_driving_vehicle_button.x + self_driving_vehicle_button.width and self_driving_vehicle_button.y <= pos[1] <= self_driving_vehicle_button.y + self_driving_vehicle_button.height:
                        pass

                    # plan path click
                    if self.path_planning_window_select == True and start_btn.x <= pos[0] <=start_btn.x + start_btn.width and start_btn.y <= pos[1] <= start_btn.y + start_btn.height:
                        logger.log("Path Planner Started")
                        
                        # begin path generation
                        if start_coord == None or end_coord == None:
                            pass
                        else:
                            # clear path
                            path = []

                            # generate grid_matrix
                            matrix = self.generate_grid_matrix(grid)
                            
                            if self.planner == 'default':
                                generated_path, runtime = generate_path(grid, matrix, start_coord[2], end_coord[2])

                            else:
                                generated_path, runtime = generate_path_custom(self.custom_planner, grid, matrix, start_coord[2], end_coord[2])
                            
                            path = generated_path
                            time_taken = runtime

                            if len(generated_path) < 0:
                                logger.log("No Optimal Path Found")

                            else:
                                logger.log("Path to Goal Generated")
                    

                    # navigate click
                    elif self.path_planning_window_select == True and navigate_button.x <= pos[0] <=navigate_button.x + navigate_button.width and navigate_button.y <= pos[1] <= navigate_button.y + navigate_button.height:
                        if len(path) != 0:
                            logger.log("Waypoint Navigation Started")
                            # generate wapoints
                            robot_pose, waypoints = generate_waypoints_v4(grid, matrix, path, start_coord, end_coord, self.WIN_WIDTH, self.WIN_HEIGHT)
                            
                            last_waypoint = waypoints[-1]
                            navigate = True
                        

                    # clear map click
                    elif self.path_planning_window_select == True and reset_btn.x <= pos[0] <=reset_btn.x + reset_btn.width and reset_btn.y <= pos[1] <= reset_btn.y + reset_btn.height:
                        logger.log("Simulation Restarted")
                        # reset window
                        robot = None
                        start_coord = None
                        end_coord = None
                        path = []
                        
                        draw_frame = False
                        self.clear_map_path_all(path, grid, start_coord, end_coord, BLUE)


                    # grid click
                    else:
                        # get clicked cell
                        if  self.path_planning_window_select == True:
                            clicked_row = pos[1] // 23
                            clicked_col = pos[0] // 23
                            cell = grid[clicked_row][clicked_col]

                            # set start cell
                            if start_coord == None and cell != end_coord:
                                try:
                                    logger.log("Robot Initialised")
                                    start_coord = cell
                                    cell[1] = RED
                                    grid_cpy[clicked_row][clicked_col][1] = RED

                                    # display robot model
                                    location_rect = start_coord[0]
                                    rect_x = location_rect.x
                                    rect_y = location_rect.y

                                    rect_center_x = rect_x + location_rect.width // 2
                                    rect_center_y = rect_y + location_rect.height // 2
                                    robot = Robot((rect_center_x, rect_center_y), pkg_resources.resource_filename('autonavsim2d', 'utils/assets/robot_circle_1.png'), wheels_dist)
                                    draw_frame = True
                                    
                                    
                                except IndexError:
                                    print('Out of bounce error')

                            # set end cell
                            elif end_coord == None and cell != start_coord:
                                try:
                                    end_coord = cell
                                    cell[1] = GREEN
                                    grid_cpy[clicked_row][clicked_col][1] = GREEN

                                    logger.log("Goal Location Set")
                                    
                                except IndexError:
                                    print('Out of bounce error')

                            # make obstacles
                            elif cell != end_coord and cell != start_coord:
                                try:
                                    cell[1] = BLACK
                                    grid_cpy[clicked_row][clicked_col][1] = BLACK
                                    
                                except IndexError:
                                    print('Out of bounce error')

                elif pygame.mouse.get_pressed()[2]:   # right click
                    pos = pygame.mouse.get_pos()

                    # get clicked cell
                    clicked_row = pos[1] // 23
                    clicked_col = pos[0] // 23
                    cell = grid[clicked_row][clicked_col]

                    # remove start coordinates
                    if self.path_planning_window_select == True and cell == start_coord:
                        start_coord = None
                        
                    # remove end coordinates
                    if self.path_planning_window_select == True and cell == end_coord:
                        end_coord = None
                    
                    if  self.path_planning_window_select == True:
                        cell[1] = GREY
                        grid_cpy[clicked_row][clicked_col][1] = GREY
            
            # get change in time
            dt = (pygame.time.get_ticks() - lasttime) / 1000
            lasttime = pygame.time.get_ticks()

            # navigate robot
            if len(waypoints) != 0 and navigate is True:
                
                if len(waypoints) != 2:
                    # get next waypoint
                    logger.log(f"Navigation - Starting at {counter}")
                    waypoint = waypoints[1]
                    first_path_point = path[1]
                    waypoint_angle = waypoint.pose.orientation.w
                    robot_angle = waypoint_angle

                    # navigate to next waypoint
                    if not compare_waypoints((robot_pose.position.x, robot_pose.position.y), (waypoint.pose.position.x,  waypoint.pose.position.y)):
                        
                        if math.degrees(waypoint_angle) == 90:
                            # move forward along y axis
                            robot.vl = 0.005 * m2p
                            robot.vr = 0.005 * m2p
                            robot.move(theta=waypoint_angle, dt=dt)
                        
                        elif math.degrees(waypoint_angle) == 0:
                            # move along x axis
                            robot.vl = 0.005 * m2p
                            robot.vr = 0.005 * m2p
                            robot.move(theta=0, dt=dt)
                        
                        elif math.degrees(waypoint_angle) == 180:
                            # move along x axis
                            robot.vl = 0.005 * m2p
                            robot.vr = 0.005 * m2p
                            robot.move(theta=waypoint_angle, dt=dt)
                        
                        elif math.degrees(waypoint_angle) == 270:
                            # move along x axis
                            robot.vl = 0.005 * m2p
                            robot.vr = 0.005 * m2p
                            robot.move(theta=waypoint_angle, dt=dt)

                        # update robot pose
                        robot_pose.position.x = round(robot.x)
                        robot_pose.position.y = round(robot.y)
                    
                    else:
                        # gotten to waypoint, increment counter
                        logger.log(f"Navigation - Reached Waypoint {counter}")
                        counter += 1

                        # clear map path
                        self.clear_map_path(path, grid, start_coord, end_coord, BLUE)
                        
                        # regenerate path and waypoints
                        path = []

                        # generate grid matrix
                        matrix_ = self.generate_grid_matrix(grid)
                        
                        # last_row, last_col = last_waypoint.pose.position.x, last_waypoint.pose.position.y
                        robot_current_matrix_cell= grid[first_path_point[0]][first_path_point[1]]
                        
                        # generated_path, runtime = generate_path(grid, matrix_, robot_current_matrix_cell[2], end_coord[2])
                        if self.planner == 'default':
                            generated_path, runtime = generate_path(grid, matrix_, robot_current_matrix_cell[2], end_coord[2])

                        else:
                            generated_path, runtime = generate_path_custom(self.custom_planner, grid, matrix_, robot_current_matrix_cell[2], end_coord[2])
                        path = generated_path

                        robot_pose, waypoints = generate_waypoints_v4(grid, matrix_, path, robot_current_matrix_cell, end_coord, self.WIN_WIDTH, self.WIN_HEIGHT)
                        time_taken = runtime
                        
                else:
                    # get 
                    logger.log(f"Navigation - Starting at {counter}")
                    waypoint = last_waypoint
                    waypoint_angle = waypoint.pose.orientation.w
                    robot_angle = waypoint_angle

                    # navigate to last waypoint
                    if not compare_waypoints((robot_pose.position.x, robot_pose.position.y), (waypoint.pose.position.x,  waypoint.pose.position.y)):
                        
                        if math.degrees(waypoint_angle) == 90:
                            # move forward along y axis
                            robot.vl = 0.005 * m2p
                            robot.vr = 0.005 * m2p
                            robot.move(theta=waypoint_angle, dt=dt)
                        
                        elif math.degrees(waypoint_angle) == 0:
                            # move along x axis
                            robot.vl = 0.005 * m2p
                            robot.vr = 0.005 * m2p
                            robot.move(theta=0, dt=dt)
                        
                        elif math.degrees(waypoint_angle) == 180:
                            # move along x axis
                            robot.vl = 0.005 * m2p
                            robot.vr = 0.005 * m2p
                            robot.move(theta=waypoint_angle, dt=dt)
                        
                        elif math.degrees(waypoint_angle) == 270:
                            # move along x axis
                            robot.vl = 0.005 * m2p
                            robot.vr = 0.005 * m2p
                            robot.move(theta=waypoint_angle, dt=dt)

                        # update robot pose
                        robot_pose.position.x = round(robot.x)
                        robot_pose.position.y = round(robot.y)
                    
                    else:
                        # last waypoint
                        robot.vl = 0 * m2p
                        robot.vr = 0 * m2p
                        robot_angle = 0
                        robot.move(theta=0, dt=dt)
                        logger.log("Goal Reached!")
                        navigate = False

            # navigate robot through smoothened path
            if len(spline_path) != 0 and navigate_2 is True:
                if counter < len(spline_path):
                    logger.log(f"Navigation - Starting at {counter}")
                    point = spline_path[counter]
                    point_angle = orientations_ep[counter]
                    robot_angle = point_angle

                    if not compare_waypoints((robot_pose.position.x, robot_pose.position.y), (point[0],  point[1])):
                        robot.vl = 0.005 * m2p
                        robot.vr = 0.005 * m2p
                        robot.move(theta=point_angle, dt=dt)

                        # update robot pose
                        robot_pose.position.x = round(robot.x)
                        robot_pose.position.y = round(robot.y)
                    
                    else:
                        # gotten to waypoint, increment counter
                        logger.log(f"Navigation - Reached Waypoint {counter}")
                        counter += 1

                else:
                    # last waypoint
                    robot.vl = 0
                    robot.vr = 0
                    robot_angle = 0
                    robot.move(theta=0, dt=dt)
                    logger.log("Goal Reached!")
                    navigate_2 = False

            # draw main window
            if self.main_window_select:
                self.draw_main_window(self.ACTIVE_WINDOW, diff_drive_btn, skid_steer_button, self_driving_vehicle_button)

            # draw path planning and nav window
            if self.path_planning_window_select:
                robot_angle_ = [robot_angle]
                self.draw_path_planning_window(self.ACTIVE_WINDOW, grid_cpy, start_btn, start_btn_color, reset_btn, reset_btn_color, time_taken, navigate_button, navigate_btn_color, robot, dt, 
                            draw_frame, robot_angle_, counter=counter, waypoints_len=len(waypoints), logger=logger, spline_path=spline_path)

            # draw path if generated
            if len(path) != 0 and draw_new_path == False:
                # draw generated path
                self.draw_path(path, grid_cpy, start_coord, end_coord, BLUE, draw_new_path)
            
            

            
        pygame.quit()




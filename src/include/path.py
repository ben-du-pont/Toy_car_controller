import csv
import numpy as np
import os

from scipy.interpolate import make_interp_spline
from shapely.geometry import LineString, Polygon, Point

from include.cubic_spline_interpolator import generate_cubic_spline

class Path:

    def __init__(self):
        
        script_directory = os.path.dirname(os.path.abspath(__file__))
        # Get the parent directory
        parent_directory = os.path.dirname(script_directory)
        # Construct the path to the file in the parent directory
        path = os.path.join(parent_directory, 'config/tracks/random_track_1.csv')
    
        # Get path to waypoints.csv


        # process the track csv file
        self.track_params = self.process_track(path, 'clockwise')

        ds = 0.05
        # column 1 is x, column 2 is y
        x = [float(row[1]) for row in self.track_params[0][0:]]
        y = [float(row[2]) for row in self.track_params[0][0:]]

        # Remove consecutive duplicates
        x, y = zip(*((x[i], y[i]) for i in range(len(x)) if i == 0 or x[i] != x[i-1]))
        
        # spline the center line
        self.px, self.py, self.pyaw, _ = generate_cubic_spline(x, y, ds)

        self.lap_count = 0
        self.end_phase = False

    def create_spline(self, points):
        points = np.vstack([points, points[0]])  # Close the loop
        x, y = points[:, 0], points[:, 1]
        t = np.linspace(0, 1, len(points))
        spline_x = make_interp_spline(t, x, k=3)
        spline_y = make_interp_spline(t, y, k=3)
        return spline_x, spline_y
    

    def convert_spline2line_track(self, blue_line, yellow_line, recessed_line1, recessed_line2):

        # set control points at 0.1 m intervals along the blue line and yellow line and connect them with a straight line

        blue_control_points = np.array([blue_line[0]])
        yellow_control_points = np.array([yellow_line[0]])
        recessed_control_points1 = np.array([recessed_line1[0]])
        recessed_control_points2 = np.array([recessed_line2[0]])
        for i in range(1, len(blue_line)):
            if np.linalg.norm(blue_line[i] - blue_control_points[-1]) > 0.1:
                blue_control_points = np.vstack([blue_control_points, blue_line[i]])
            
        for i in range(1, len(yellow_line)):
            if np.linalg.norm(yellow_line[i] - yellow_control_points[-1]) > 0.1:
                yellow_control_points = np.vstack([yellow_control_points, yellow_line[i]])


        for i in range(1, len(recessed_line1)):
            if np.linalg.norm(recessed_line1[i] - recessed_control_points1[-1]) > 0.1:
                recessed_control_points1 = np.vstack([recessed_control_points1, recessed_line1[i]])
        
        for i in range(1, len(recessed_line2)):
            if np.linalg.norm(recessed_line2[i] - recessed_control_points2[-1]) > 0.1:
                recessed_control_points2 = np.vstack([recessed_control_points2, recessed_line2[i]])
        
        blue_piecewise_perimeter = np.array([blue_control_points[0]])
        yellow_piecewise_perimeter = np.array([yellow_control_points[0]])
        
        recessed_line1_perimeter = np.array([recessed_control_points1[0]])
        recessed_line2_perimeter = np.array([recessed_control_points2[0]])
        
        for i in range(1, len(blue_control_points)):
            blue_piecewise_perimeter = np.vstack([blue_piecewise_perimeter, blue_control_points[i]])

        for i in range(1, len(yellow_control_points)):
            yellow_piecewise_perimeter = np.vstack([yellow_piecewise_perimeter, yellow_control_points[i]])

        for i in range(1, len(recessed_control_points1)):
            recessed_line1_perimeter = np.vstack([recessed_line1_perimeter, recessed_control_points1[i]])

        for i in range(1, len(recessed_control_points2)):
            recessed_line2_perimeter = np.vstack([recessed_line2_perimeter, recessed_control_points2[i]])

        return blue_piecewise_perimeter, yellow_piecewise_perimeter, recessed_line1_perimeter, recessed_line2_perimeter

    def process_track(self, file_path, track_direction):
        # Initialize lists for blue, yellow, and orange cones
        blue_cones = []
        yellow_cones = []
        orange_cones = []

        # Open the file
            # Read the CSV file
        with open(file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)  # Skip the header row if there is one
            for row in reader:
                if len(row) < 3:
                    continue  # Skip rows that don't have enough data
                cone_type, x, y = row[0], float(row[1]), float(row[2])
                if cone_type == 'blue':
                    blue_cones.append((x, y))
                elif cone_type == 'yellow':
                    yellow_cones.append((x, y))
                elif cone_type == 'big_orange':
                    orange_cones.append((x, y))

        # Convert lists to numpy arrays
        blue_cones = np.array(blue_cones)
        yellow_cones = np.array(yellow_cones)
        orange_cones = np.array(orange_cones)


        # Create B-Splines for the track perimeter
        spline_x_blue, spline_y_blue = self.create_spline(blue_cones)
        spline_x_yellow, spline_y_yellow = self.create_spline(yellow_cones)

        # Generate points for the splines
        t_new = np.linspace(0, 1, 1000)
        blue_line = np.array([spline_x_blue(t_new), spline_y_blue(t_new)]).T
        yellow_line = np.array([spline_x_yellow(t_new), spline_y_yellow(t_new)]).T


        # initailize control points along the blue line at 1 meter intervals
        control_points = np.array([blue_line[0]])
        for i in range(1, len(blue_line)):
            if np.linalg.norm(blue_line[i] - control_points[-1]) > 0.1:
                control_points = np.vstack([control_points, blue_line[i]])
        
        # define waypoints as the midpoint between control points on the blue line and the closest point on the yellow line
        center_line = []
        # define recessed line1 adn recessed line2 on control point 50cm away from the blue and yellow line on the normal vector
        recessed_line1 = []
        recessed_line2 = []
        recess_distance = 0.6
        for i in range(len(control_points)):
            distances = np.linalg.norm(yellow_line - control_points[i], axis=1)
            closest_point = yellow_line[np.argmin(distances)]
            center_line.append((control_points[i] + closest_point) / 2)
            # define recess_point1 adn  recess_point2 on control point 50cm away from the blue and yellow line on the normal vector
            normal_vector = (control_points[i] - closest_point) / np.linalg.norm(control_points[i] - closest_point)
            recess_point1 = control_points[i] - recess_distance * normal_vector
            # define recess_point 2 in relation to the intersection point of the normal vector and the yellow line
            recess_point2 = closest_point + recess_distance * normal_vector


            recessed_line1.append(recess_point1)
            recessed_line2.append(recess_point2)

        center_line = np.array(center_line)
        recessed_line1 = np.array(recessed_line1)
        recessed_line2 = np.array(recessed_line2)



        # Sort the waypoints according to the track direction
        if track_direction == 'counterclockwise':
            # Reverse the center line array if the race direction is counterclockwise
            center_line = center_line[::-1]

        # Find the index of the start/finish line
        start_finish_line = np.array([orange_cones[0], orange_cones[1]])
        distances_to_start_line = np.linalg.norm(center_line - start_finish_line[0], axis=1)
        start_index = np.argmin(distances_to_start_line)

        # calculate the length of the track on the center line
        track_length = np.sum(np.linalg.norm(np.diff(center_line, axis=0), axis=1))
        print('Track length: {:.2f} m'.format(track_length))
        # count the points on the centre line 
        num_waypoints = len(center_line)


        # Roll the center line array so the start line is the first element
        center_line = np.roll(center_line, -start_index, axis=0)

        # Set waypoints along the center line
        waypoints_indices = np.linspace(0, len(center_line) - 1, num_waypoints, dtype=int)
        waypoints = center_line[waypoints_indices]

        waypoint_distances = np.cumsum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1))
        waypoint_distances = np.insert(waypoint_distances, 0, 0)  # Add a zero to the beginning of the array

        # Return a structured array similar to a dataframe
        dtype = [('index', int), ('x', float), ('y', float)]
        structured_waypoints = np.array([(i, x, y) for i, (x, y) in enumerate(waypoints, start=0)], dtype=dtype)

        waypoint_progress = {}
        # iterate over the waypoints
        for i in range(len(waypoints)):
            waypoint_progress[i] = waypoint_distances[i] / track_length
            
        # convert line to piecewise line
        blue_line, yellow_line, recessed_line1, recessed_line2 = self.convert_spline2line_track(blue_line, yellow_line, recessed_line1, recessed_line2)

        # convert piecewise line to shapely LineString
        blue_line = LineString(blue_line)
        yellow_line = LineString(yellow_line)
        recessed_line1 = LineString(recessed_line1)
        recessed_line2 = LineString(recessed_line2)

        # define the track in shapely as blue line minus yellow line
        blue_line_coords = list(blue_line.coords)
        yellow_line_coords = list(yellow_line.coords)

        track_polygon = Polygon(np.vstack((blue_line_coords, yellow_line_coords[::-1])))

        return structured_waypoints, blue_line, yellow_line, center_line, start_finish_line, recessed_line1, recessed_line2, waypoint_progress, track_polygon


    def get_track_width(self, point_x, point_y, blue_line, yellow_line):
        point = Point(point_x, point_y)
        blue_line_coords = list(blue_line.coords)
        yellow_line_coords = list(yellow_line.coords)

        closest_blue_point = min(blue_line_coords, key=lambda x: Point(x).distance(point))
        closest_yellow_point = min(yellow_line_coords, key=lambda x: Point(x).distance(point))
        track_width = np.linalg.norm(np.array(closest_blue_point) - np.array(closest_yellow_point))
        intersection_line = LineString([closest_blue_point, closest_yellow_point])

        return track_width, intersection_line
    
    def get_next_waypoint(self, position_x, position_y, center_line, structured_waypoints, blue_line, yellow_line):
        
        car_point = Point(position_x, position_y)
        center_line = LineString(center_line)
        try:
            # Attempt to find the closest point on the line to the car_point
            closest_point = center_line.interpolate(center_line.project(car_point))
        except ValueError as e:
            # Handle the case where the point cannot be projected onto the line
            print("Error:", e)
            return 0
        closest_point = center_line.interpolate(center_line.project(car_point))
        closest_waypoint = min(structured_waypoints, key=lambda x: Point(x[1], x[2]).distance(closest_point))
        next_closest_waypoint = (closest_waypoint[0] + 1) % len(structured_waypoints)
        next_closest_waypoint_xy = (structured_waypoints[next_closest_waypoint][1], structured_waypoints[next_closest_waypoint][2])
        closest_waypoint_index = closest_waypoint[0]
        next_closest_waypoint_index = structured_waypoints[next_closest_waypoint][0]
        track_width_at_closest_waypoint, intersection_line = self.get_track_width(structured_waypoints[closest_waypoint_index][1], structured_waypoints[closest_waypoint_index][2], blue_line, yellow_line)
        # define test_line from car_x, car_y to next_closest_waypoint xy, if the test_line intersects with the intersection_line, then the next waypoint is the next_closest_waypoint
        
        test_line = LineString([(position_x, position_y), next_closest_waypoint_xy])

        if test_line.intersects(intersection_line):
            return closest_waypoint_index
        else:
            return next_closest_waypoint_index


    def get_a_waypoint(self, position_x, position_y, center_line, structured_waypoints, blue_line, yellow_line, lookahead_distance):
        
        car_point = Point(position_x, position_y)
        center_line = LineString(center_line)

        # Find the closest point on the center line to the car
        try:
            closest_point = center_line.interpolate(center_line.project(car_point))
        except ValueError as e:
            print("Error:", e)
            return 0
        # Find the closest waypoint to the closest point on the center line
        closest_waypoint = min(structured_waypoints, key=lambda x: Point(x['x'], x['y']).distance(closest_point))
        
        # Calculate the distance along the path from the closest waypoint
        accumulated_distance = 0
        current_index = closest_waypoint['index']
        target_index = current_index  # Initialize target index to the closest waypoint's index
        
        # Iterate through waypoints to find the one that is lookahead_distance away
        while accumulated_distance < lookahead_distance and current_index < len(structured_waypoints) - 1:
            current_waypoint = Point(structured_waypoints[current_index]['x'], structured_waypoints[current_index]['y'])
            next_waypoint = Point(structured_waypoints[current_index + 1]['x'], structured_waypoints[current_index + 1]['y'])
            segment_distance = current_waypoint.distance(next_waypoint)
            accumulated_distance += segment_distance
            
            if accumulated_distance < lookahead_distance:
                current_index += 1
            target_index = current_index

        
        return target_index
    
    def calculate_distance_to_boundaries(self, x, y, blue_line, yellow_line):
        """
        Calculate the distance to the left and right boundaries from a given (x, y) position.

        Args:
        - x, y (float): The x and y coordinates of the position.
        - blue_line (LineString): The LineString representing the blue track boundary.
        - yellow_line (LineString): The LineString representing the yellow track boundary.

        Returns:
        - distance_left (float): The distance to the left boundary.
        - distance_right (float): The distance to the right boundary.
        """
        point = Point(x, y)
        distance_left = point.distance(blue_line)
        distance_right = point.distance(yellow_line)
        return distance_left, distance_right


    def increment_lap_count(self, x, y):
        waypoint = self.get_next_waypoint(x, y, self.track_params[3], self.track_params[0], self.track_params[1], self.track_params[2])
        
        if waypoint > len(self.track_params[0]) - 10 and self.end_phase == False:
            self.end_phase = True
        elif waypoint <= len(self.track_params[0]) - 10:
            if self.end_phase == True:
                self.lap_count += 1
            self.end_phase = False


    def check_collision(self, x, y):
        distance_left, distance_right = self.calculate_distance_to_boundaries(x, y, self.track_params[1], self.track_params[2])
        track_with, _ = self.get_track_width(x, y, self.track_params[1], self.track_params[2])

        if distance_left > track_with or distance_right > track_with:
            return True
        else:
            return False
        
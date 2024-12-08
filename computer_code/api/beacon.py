import numpy as np
from scipy.optimize import fsolve

# Define beacon positions
beacons = [
    [1, 1, 0],  # Beacon A
    [-1, 1, 0], # Beacon B
    [0, -1, 0], # Beacon C
]

# Function to compute the system of equations
def equations(p, beacons, distances):
    x, y, z = p
    eq1 = (x - beacons[0][0])**2 + (y - beacons[0][1])**2 + (z - beacons[0][2])**2 - distances[0]**2
    eq2 = (x - beacons[1][0])**2 + (y - beacons[1][1])**2 + (z - beacons[1][2])**2 - distances[1]**2
    eq3 = (x - beacons[2][0])**2 + (y - beacons[2][1])**2 + (z - beacons[2][2])**2 - distances[2]**2
    return [eq1, eq2, eq3]

# Main function to calculate the point using triangulation
def triangulate(beacons, distances):
    # Initial guess for the position (e.g., the centroid of the beacons)
    initial_guess = np.mean(beacons, axis=0)

    # Solve the system of equations
    solution = fsolve(equations, initial_guess, args=(beacons, distances))
    
    # Check the solution and ensure it's valid
    return solution

# Example usage
distances = [4, 4, 4]  # Distances to the three beacons
coordinates = triangulate(beacons, distances)
print("Calculated coordinates:", coordinates)
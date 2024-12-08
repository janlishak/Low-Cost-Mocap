import math

print("running distance.py")
def euclidean_distance(point1, point2):
    return math.sqrt(
        (point1[0] - point2[0])**2 + 
        (point1[1] - point2[1])**2 + 
        (point1[2] - point2[2])**2
    )

# Example points
point1 = [ 0.14845797834329288, 0.015891865436492125, 0.002154076038069608 ]
point2 = [ -0.45462326113526674, 0.5730211411462696, -0.0869808515392183 ]

distance = euclidean_distance(point1, point2)
print(f"The Euclidean distance between the points is: {distance}")
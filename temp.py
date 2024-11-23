# Corrected positions where the centroids of each triangle are aligned at the origin
import numpy as np
import matplotlib.pyplot as plt
r = 1
sqrt3 = np.sqrt(3)

# Compute the vertical offset for centroids alignment
offsets = [0, -sqrt3 * r, -2 * sqrt3 * r]  # Offsets for 1st, 2nd, and 3rd rows

# Define circle centers for proper triangle alignment
circle_centers_with_triangles = [
    # First row (1 circle, directly at the centroid)
    (0, offsets[0]),

    # Second row (3 circles forming a triangle with centroid aligned at origin)
    (-r, offsets[1]), (0, offsets[1]), (r, offsets[1]),

    # Third row (6 circles forming a triangle with centroid aligned at origin)
    (-2 * r, offsets[2]), (-r, offsets[2]), (0, offsets[2]),
    (r, offsets[2]), (2 * r, offsets[2])
]

# Plot the circles and the triangles
plt.figure(figsize=(6, 6))

# Draw circles
for center in circle_centers_with_triangles:
    circle = plt.Circle(center, r, color='blue', fill=False, linewidth=1.5)
    plt.gca().add_artist(circle)
    plt.plot(center[0], center[1], 'ro')  # Mark the center
    plt.text(center[0], center[1], f"{center}", fontsize=8, ha='center', color='green')

# Draw the triangles for each row
# First row: no triangle (1 circle only)
# Second row triangle
plt.plot(
    [-r, 0, r, -r], [offsets[1], offsets[1], offsets[1], offsets[1]],
    color='purple', linestyle='--', linewidth=1.5, label="Second Row Triangle"
)
# Third row triangle
plt.plot(
    [-2 * r, 0, 2 * r, -2 * r], [offsets[2], offsets[2], offsets[2], offsets[2]],
    color='orange', linestyle='--', linewidth=1.5, label="Third Row Triangle"
)

# Add labels and grid
plt.axhline(0, color='black', linewidth=0.5, linestyle='--')
plt.axvline(0, color='black', linewidth=0.5, linestyle='--')
plt.grid(color='gray', linestyle='--', linewidth=0.5)
plt.gca().set_aspect('equal', adjustable='box')
plt.title("Pyramid of Circles with Centroids of Rows Aligned to Origin")
plt.xlabel("x-axis")
plt.ylabel("y-axis")
plt.xlim(-4, 4)
plt.ylim(-6, 1)
plt.legend()
plt.show()

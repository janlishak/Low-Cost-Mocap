import numpy as np
import plotly.graph_objects as go

matrix = np.random.rand(20, 20, 20)
x, y, z = np.indices((20, 20, 20))

# Define the target point (5, 5, 5)
target_x, target_y, target_z = 10, 10, 10

# Compute the Euclidean distance from the point (5, 5, 5)
distance = np.sqrt((x - target_x)**2 + (y - target_y)**2 + (z - target_z)**2)
matrix = ((distance > 4) & (distance < 5)).astype(int)
# Step 3: Flatten the grid and matrix for plotting
x_flat = x.flatten()
y_flat = y.flatten()
z_flat = z.flatten()
values = matrix.flatten()

# Step 4: Normalize the values for opacity
opacity = 1 - values  # Higher values are less transparent

# Step 5: Define the color scale (grayscale)
colorscale = [[0, "rgba(0,0,0,0)"], [1, "rgba(0,0,0,255)"]]
# Step 6: Create the 3D scatter plot
fig = go.Figure()

fig.add_trace(go.Scatter3d(
    x=x_flat,
    y=y_flat,
    z=z_flat,
    mode='markers',
    marker=dict(
        size=5,  # Size of the points
        color=values,  # Color based on the matrix value
        colorscale=colorscale,  # Grayscale colormap
        showscale=True  # Display color scale
    )
))


# Step 7: Customize layout
fig.update_layout(
    scene=dict(
        xaxis_title='X',
        yaxis_title='Y',
        zaxis_title='Z'
    ),
    title='3D Matrix Visualization with Plotly'
)

# Step 8: Display the figure
fig.show()

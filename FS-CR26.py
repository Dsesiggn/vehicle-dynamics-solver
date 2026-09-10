import numpy as np
import plotly.graph_objects as go

# 1. Define the CR26 Hardpoints (Front Left)
cr26_front_left = {
    "uca_front": np.array([263.53, -127.00, 263.53]),
    "uca_rear":  np.array([232.43, 127.00, 248.77]),
    "uca_outer": np.array([482.60, 9.12, 285.98]),
    
    "lca_front": np.array([215.90, -117.48, 120.65]),
    "lca_rear":  np.array([215.90, 123.42, 127.00]),
    "lca_outer": np.array([533.40, -3.18, 119.13]),
    
    "tie_rod_inner": np.array([219.08, -69.85, 151.99]),
    "tie_rod_outer": np.array([542.93, -73.03, 171.45]),
    
    "wheel_center":  np.array([558.80, 0.00, 203.20])
}

# 2. Automatically Generate Front Right points by mirroring the X-axis
cr26_front_right = {}
for key, value in cr26_front_left.items():
    cr26_front_right[key] = np.array([-value[0], value[1], value[2]])

# Helper functions for plotting
def extract_xyz(points):
    return [p[0] for p in points], [p[1] for p in points], [p[2] for p in points]

def create_paths(corner):
    uca = [corner["uca_front"], corner["uca_outer"], corner["uca_rear"]]
    lca = [corner["lca_front"], corner["lca_outer"], corner["lca_rear"]]
    tie_rod = [corner["tie_rod_inner"], corner["tie_rod_outer"]]
    upright = [corner["uca_outer"], corner["wheel_center"], corner["lca_outer"], corner["tie_rod_outer"], corner["wheel_center"]]
    kingpin = [corner["uca_outer"], corner["lca_outer"]]
    return uca, lca, tie_rod, upright, kingpin

# 3. ONLY plot the 3D model if this specific script is run
if __name__ == "__main__":
    fig = go.Figure()

    # Loop through both sides and plot them
    corners = [('Left', cr26_front_left, 'blue'), ('Right', cr26_front_right, 'red')]
    
    for side_name, side_dict, color_mod in corners:
        uca, lca, tr, up, kp = create_paths(side_dict)
        
        x, y, z = extract_xyz(uca)
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines+markers', name=f'UCA {side_name}', line=dict(color=color_mod, width=5)))
        
        x, y, z = extract_xyz(lca)
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines+markers', name=f'LCA {side_name}', line=dict(color=color_mod, width=5)))
        
        x, y, z = extract_xyz(tr)
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines+markers', name=f'Tie Rod {side_name}', line=dict(color='green', width=5)))
        
        x, y, z = extract_xyz(up)
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines+markers', name=f'Upright {side_name}', line=dict(color='gray', width=3)))
        
        x, y, z = extract_xyz(kp)
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines', name=f'Kingpin {side_name}', line=dict(color='orange', width=4, dash='dash')))

    # Add chassis bulkheads to connect the left and right mounts
    chassis_links = [
        [cr26_front_left["uca_front"], cr26_front_right["uca_front"]],
        [cr26_front_left["uca_rear"], cr26_front_right["uca_rear"]],
        [cr26_front_left["lca_front"], cr26_front_right["lca_front"]],
        [cr26_front_left["lca_rear"], cr26_front_right["lca_rear"]],
        [cr26_front_left["tie_rod_inner"], cr26_front_right["tie_rod_inner"]]
    ]

    for link in chassis_links:
        x, y, z = extract_xyz(link)
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines', showlegend=False, line=dict(color='black', width=2, dash='dot')))

    fig.update_layout(
        title='CR26 Full Front Suspension Geometry',
        scene=dict(
            xaxis_title='X (Lateral - Outboard)',
            yaxis_title='Y (Longitudinal - Fwd/Rwd)',
            zaxis_title='Z (Vertical - Up/Down)',
            aspectmode='data' 
        ),
        width=1000,
        height=800
    )
    fig.show()
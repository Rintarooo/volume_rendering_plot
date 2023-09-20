import numpy as np
import plotly.graph_objects as go

def get_trace_cube(pos_cube, mint, maxt, rgba):
    pos_x, pos_y, pos_z = pos_cube# 3 dimentional tuple
    # create points
    x, y, z = np.meshgrid(
        # np.linspace(mint, maxt, 2), 
        # np.linspace(mint, maxt, 2), 
        # np.linspace(mint, maxt, 2),
        np.linspace(mint+pos_x, maxt+pos_x, 2), 
        np.linspace(mint+pos_y, maxt+pos_y, 2), 
        np.linspace(mint+pos_z, maxt+pos_z, 2),
    )
    # logger.debug(f"plotly cube x: {x}")
    x = x.flatten()
    # logger.debug(f"plotly cube x flatten: {x}")
    y = y.flatten()
    z = z.flatten()
    
    # return go.Mesh3d(x=x, y=y, z=z, alphahull=1, flatshading=True, rgba=rgba, lighting={'diffuse': 0.1, 'specular': 2.0, 'roughness': 0.5})
    trace_cube = [go.Mesh3d(x=x, y=y, z=z, alphahull=1.0, color=rgba, name = "cube")]
    return trace_cube


if __name__ == "__main__":
    a = 3
    pos_cube_center = (0,0,9)#(0,0,7)#(0,0,-3)
    mint, maxt = -2.0, 2.0#-3.0, 3.0#-1.0, 1.0#-0.5, 0.5
    rgba = 'rgba(0,0,0,0.2)'
    trace_cube = get_trace_cube(pos_cube_center, mint, maxt, rgba)


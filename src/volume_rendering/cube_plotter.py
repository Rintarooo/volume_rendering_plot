import numpy as np
import plotly.graph_objects as go

from src.utils.logger_global import logger
from src.utils.base_plotter import BasePlotter

# poetry run python -m src.volume_rendering.cube_plotter

class CubePlotter(BasePlotter):
    def __init__(self, pos_cube_center, mint, maxt, rgba):
        super().__init__()
        self.pos_cube_center = pos_cube_center
        self.mint = mint
        self.maxt = maxt
        self.rgba = rgba

    def get_trace_cube(self):
        pos_x, pos_y, pos_z = self.pos_cube_center# 3 dimentional tuple
        # create points
        x, y, z = np.meshgrid(
            # np.linspace(mint, maxt, 2), 
            # np.linspace(mint, maxt, 2), 
            # np.linspace(mint, maxt, 2),
            np.linspace(self.mint+pos_x, self.maxt+pos_x, 2), 
            np.linspace(self.mint+pos_y, self.maxt+pos_y, 2), 
            np.linspace(self.mint+pos_z, self.maxt+pos_z, 2),
        )
        # logger.debug(f"plotly cube x: {x}")
        x = x.flatten()
        # logger.debug(f"plotly cube x flatten: {x}")
        y = y.flatten()
        z = z.flatten()
        
        # return go.Mesh3d(x=x, y=y, z=z, alphahull=1, flatshading=True, rgba=rgba, lighting={'diffuse': 0.1, 'specular': 2.0, 'roughness': 0.5})
        trace_cube = go.Mesh3d(x=x, y=y, z=z, alphahull=1.0, color=self.rgba, name = "cube")
        # self.fig.add_trace(trace_cube)
        # self.fig.show()
        return trace_cube


if __name__ == "__main__":
    a = 3
    pos_cube_center = (0,0,9)#(0,0,7)#(0,0,-3)
    mint, maxt = -2.0, 2.0#-3.0, 3.0#-1.0, 1.0#-0.5, 0.5
    rgba = 'rgba(0,0,0,0.2)'
    cube_plotter = CubePlotter(pos_cube_center, mint, maxt, rgba)
    trace_cube = cube_plotter.get_trace_cube()


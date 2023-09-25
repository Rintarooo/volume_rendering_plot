import numpy as np
import plotly.graph_objects as go

from src.utils.logger_global import logger
from src.utils.base_plotter import BasePlotter
from src.utils.np_vec import normalize

# poetry run python -m src.utils.camera_plotter

class CameraPlotter(BasePlotter):
    def __init__(self, w_, h_, cam_pos, cam_lookat, cam_up, fov = 45, plot_scale=1.0):
        super().__init__()
        self.cam_pos = cam_pos
        self.cam_lookat = cam_lookat
        self.cam_up = cam_up
        # self.look_dir = normalize(self.cam_lookat - self.cam_pos)# look direction
        # self.cam_right = np.cross(self.look_dir, self.cam_up)
        self.cam_right = normalize(np.cross(self.cam_lookat, self.cam_up))
        
        self.cam_pyramid, self.lines = self.get_pyramid()
        

        self.w_ = w_
        self.h_ = h_
        # カメラからスクリーンまでの距離
        self.fl_x = w_ / (2. * np.tan(np.radians(fov) * 0.5))
        self.fl_y = h_ / (2. * np.tan(np.radians(fov) * 0.5))


        self.plot_scale = plot_scale
        # logger.debug(f"self.plot_scale: {self.plot_scale}")
        self.cam_pyramid_world_lis = []
        self.cam_pos_world_lis = []
        self.cam_up_world_lis = []
        self.cam_lookat_world_lis = []
        self.cam_right_world_lis = []
        self.cam_plot_name_lis = []
        self.ray_dirs_world_lis = []

        # self.fig = fig


    def get_pyramid(self):
        # pyramid verticies
        # points = [[ 0,  w_/2,  w_/2, -w_/2, -w_/2], 
        #                [ 0,  h_/2, -h_/2, -h_/2, h_/2],
        #                [ 0,  -1.0,  -1.0,  -1.0,  -1.0]] # towards -z
        # points = [[ 0,  1., 1., -1., -1.],  
        #         [ 0,  1., -1., -1., 1.],
        #         [ 0,  1., 1., 1., 1.]] # towards z
        points = [[ 0,  1., 1., -1., -1.],  
                [ 0,  1., -1., -1., 1.],
                [ 0,  -1., -1., -1., -1.]] # towards -z
        
        # pyramid lines
        lines = [[1, 2], [2, 3], [3, 4], [4, 1], 
                [0, 1], [0, 2], [0, 3], [0, 4]]
        return points, lines


    def cam2world(self, M_ext):
        """
        input
        M_ext: 4x4 mat
        M_ext convert 3d vector x from world to cam.
        x_c = M_ext * x_w


        return
        R: 3x3 mat
        t: 3x1 vec

        """
        R = M_ext[:3, :3] # rotation matrix
        t = M_ext[:3, 3] # translation vector

        # cam = np.array(self.cam_pyramid) * self.cam_size
        # cam[0, :] *= np.tan(self.ca_x/2)
        # cam[1, :] *= np.tan(self.ca_y/2)

        cam_pyramid_world = np.array(self.cam_pyramid)
        # cam *= self.plot_scale
        # logger.debug(f"cam_pyramid_world: {cam_pyramid_world}")
        ## [x',y',z'] = [x/fl_x, y/fl_y, 1]
        cam_pyramid_world[0,:] *= 1./self.fl_x
        cam_pyramid_world[1,:] *= 1./self.fl_y
        # cam_pyramid_world[2,:] *= self.fl_y

        # logger.debug(f"after cam_pyramid_world: {cam_pyramid_world}")


        # cam = -((R @ cam).T - t).T        
        # cam = R.T @ cam - R.T @ t

        num_points = len(self.cam_pyramid[0])
        for i in range(num_points):
            # print("cam[:,i]: ", cam[:,i])
            cam_pyramid_world[:,i] = R.T @ cam_pyramid_world[:,i] - R.T @ t

        # cam = R.T @ cam - R.T @ t
        # np.linalg.inv

        cam_pos_world = R.T @ self.cam_pos - R.T @ t
        cam_lookat_world = R.T @ self.cam_lookat - R.T @ t
        cam_up_world = R.T @ self.cam_up - R.T @ t
        cam_right_world = R.T @ self.cam_right - R.T @ t
        return cam_pyramid_world, cam_pos_world, cam_lookat_world, cam_up_world, cam_right_world

    def add_cam(self, M_ext, plot_color, plot_name):
        logger.debug(f"plot_name: {plot_name}, M_ext: {M_ext}")
        cam_pyramid_world, cam_pos_world, cam_lookat_world, cam_up_world, cam_right_world = self.cam2world(M_ext)
        logger.debug(f"plot_name: {plot_name}, cam_pos_world: {cam_pos_world}")
        # self.cam_pyramid_world_lis.append((cam_pyramid_world, plot_color, plot_name))
        self.cam_pyramid_world_lis.append(cam_pyramid_world)
        self.cam_pos_world_lis.append(cam_pos_world)
        self.cam_up_world_lis.append(cam_up_world)
        self.cam_lookat_world_lis.append(cam_lookat_world)
        self.cam_right_world_lis.append(cam_right_world)
        self.cam_plot_name_lis.append(plot_name)
    
    def update_cam(self, M_ext, plot_color, plot_name):
        cam_pyramid_world = self.cam2world(M_ext)
        del self.cam_pyramid_world_lis[-1]
        # self.cam_lis.append((cam, plot_color, plot_name))
    
    def add_trace_cam_coord(self):
        assert len(self.cam_pos_world_lis) > 0, "you should call method, add_cam"
        for i in range(len(self.cam_pos_world_lis)):
            cam_pos = self.cam_pos_world_lis[i]
            cam_up = self.cam_up_world_lis[i]
            cam_lookat = self.cam_lookat_world_lis[i]
            cam_right = self.cam_right_world_lis[i]
            plot_name = self.cam_plot_name_lis[i]
            trace1 = {
                "line": {"width": 9}, 
                "mode": "lines", 
                "name": plot_name, 
                "type": "scatter3d", 
                "x": [cam_pos[0], cam_up[0]], 
                "y": [cam_pos[1], cam_up[1]], 
                "z": [cam_pos[2], cam_up[2]], 
                "marker": {"color": "rgb(0, 200, 0)"}, 
                "showlegend": False
            }
            trace2 = {
                "name": "cam_up", 
                "type": "cone", 
                "u": [cam_up[0]-cam_pos[0]], # 矢印の終点のx座標
                "v": [cam_up[1]-cam_pos[1]], 
                "w": [cam_up[2]-cam_pos[2]], 
                "x": [cam_up[0]], # 矢印の始点のx座標
                "y": [cam_up[1]], 
                "z": [cam_up[2]], 
                "sizeref": 0.1, 
                "lighting": {"ambient": 0.8}, 
                "sizemode": "scaled", 
                "hoverinfo": "x+y+z+name", 
                "colorscale": [[0.0, 'rgb(0,200,0)'],[1.0, 'rgb(0,200,0)']],
                "showscale": False, 
                "autocolorscale": False
            }
            trace3 = {
                "line": {"width": 9}, 
                "mode": "lines", 
                "name": plot_name, 
                "type": "scatter3d", 
                "x": [cam_pos[0], cam_right[0]], 
                "y": [cam_pos[1], cam_right[1]], 
                "z": [cam_pos[2], cam_right[2]], 
                "marker": {"color": "rgb(200, 0, 0)"}, 
                "showlegend": False
            }
            trace4 = {
                "name": "cam_right", 
                "type": "cone", 
                "u": [cam_right[0]-cam_pos[0]], # 矢印の終点のx座標
                "v": [cam_right[1]-cam_pos[1]], 
                "w": [cam_right[2]-cam_pos[2]], 
                "x": [cam_right[0]], # 矢印の始点のx座標
                "y": [cam_right[1]], 
                "z": [cam_right[2]], 
                "sizeref": 0.1, 
                "lighting": {"ambient": 0.8}, 
                "sizemode": "scaled", 
                "hoverinfo": "x+y+z+name", 
                "colorscale": [[0.0, 'rgb(200,0,0)'],[1.0, 'rgb(200,0,0)']],
                "showscale": False, 
                "autocolorscale": False
            }
            trace5 = {
                "line": {"width": 9}, 
                "mode": "lines", 
                "name": plot_name, 
                "type": "scatter3d", 
                "x": [cam_pos[0], cam_lookat[0]], 
                "y": [cam_pos[1], cam_lookat[1]], 
                "z": [cam_pos[2], cam_lookat[2]], 
                "marker": {"color": "rgb(0, 0, 200)"}, 
                "showlegend": False
            }
            trace6 = {
                "name": "cam_lookat", 
                "type": "cone", 
                "u": [cam_lookat[0]-cam_pos[0]], # 矢印の終点のx座標
                "v": [cam_lookat[1]-cam_pos[1]], 
                "w": [cam_lookat[2]-cam_pos[2]], 
                "x": [cam_lookat[0]], # 矢印の始点のx座標
                "y": [cam_lookat[1]], 
                "z": [cam_lookat[2]], 
                "sizeref": 0.1, 
                "lighting": {"ambient": 0.8}, 
                "sizemode": "scaled", 
                "hoverinfo": "x+y+z+name", 
                "colorscale": [[0.0, 'rgb(0,0,200)'],[1.0, 'rgb(0,0,200)']],
                "showscale": False, 
                "autocolorscale": False
            }
            trace_cam_coord = [trace1, trace2, trace3, trace4, trace5, trace6]
            self.fig.add_traces(trace_cam_coord)


    def add_trace_cam_screen(self):
        color='blue'
        # name='cam-screen'
        # square draw
        for i in range(len(self.cam_pyramid_world_lis)):
            cam = self.cam_pyramid_world_lis[i]
            name = self.cam_plot_name_lis[i]
            trace_screen_lines = []
            for i, j in self.lines:
                x, y, z = [[axis[i], axis[j]] for axis in cam]
                # logger.debug(f"lines: [{x},{y},{z}]")
                # x, y, z = [[x, y, z][i] + cam_pos[i] for i in range(3)]
                # logger.debug(f"lines + cam_pos: [{x},{y},{z}]")
                trace = go.Scatter3d(x=x, y=y, z=z, mode='lines', line_width=2, line_color=color, name=name)
                trace_screen_lines.append(trace)
            x, y, z = cam[:, 1:]
            # logger.debug(f"x,y,z: {x}, {y}, {z}, name: {name}")
            trace_screen_mesh = [go.Mesh3d(x=x, y=y, z=z, opacity=0.2, color=color, name=name)]
            self.fig.add_traces(trace_screen_lines + trace_screen_mesh)

        # boarder in square draw
        # for i in range(len(pxl_boarders_x)):
        #     x, y, z = [[pxl_boarders_x[i], pxl_boarders_y[j]] for axis in cam]
        # trace_boarder = go.Scatter3d(x=pxl_boarders_x, y=pxl_boarders_y, z=[cam[-1][-1] for _ in range(len(pxl_boarders_x))], mode='lines', line_width=2, line_color=color, name="pixel")
        # grid = np.mgrid[-width/2:(width/2)+0.01:1.0, -width/2:(width/2)+0.01:1.0]
        # print(grid.shape)
        # print([0.1 for _ in range(grid.shape[1]))])
        # trace_boarder = go.Scatter3d(x=grid[0], y=grid[1], z=[cam[-1][-1] for _ in range(len(grid[0]))], mode='markers+lines', line_width=2, line_color=color, name="pixel")
        # trace_boarder = go.Scatter3d(x=grid[0], y=grid[1], z=[0.1 for _ in range(len(grid[0]))], mode='markers', line_width=2, line_color=color, name="pixel")
        # 2次元グリッドの座標を定義
        # x = [-width/2 + d for d in range(int(width/2))]#[-1.5, 0, 1.5]
        # y = [-width/2 + d for d in range(int(width/2))]#[-1.5, 0, 1.5]


        # dw = (float)(2./width)# 2 = 1.- (-1.)
        # dh = (float)(2./height)
        # delta = 10**-5
        # x = np.arange(-1., 1.+delta, 2./dw, dtype = float)
        # y = np.arange(-1., 1.+delta, 2./dh, dtype = float)
        # print(x)


        # x = [-width/2 + d for d in range(width+1)]#[-1.5, -0.5, 0.5, 1.5]
        # y = [-height/2 + d for d in range(height+1)]#[-1.5, -0.5, 0.5, 1.5]

        # # print(x, y)
        # for i in range(width+1):
        #     for j in range(height+1):
        #         trace_gridx = go.Scatter3d(x=[x[j]]*(width+1), y=y, z=[cam[-1][-1] for _ in range(width+1)], mode='lines', line_width=2, line_color=color, name="pixel")
        #         fig.add_trace(trace_gridx)
        #         trace_gridy = go.Scatter3d(x=x, y=[y[i]]*(height+1), z=[cam[-1][-1] for _ in range(height+1)], mode='lines', line_width=2, line_color=color, name="pixel")
        #         fig.add_trace(trace_gridy)
        # fig.add_trace(trace_boarder)

        # x, y, z = cam[:, 1:]
        # mesh = go.Mesh3d(x=x, y=y, z=z, opacity=0.5, color=color, name=name)
        # fig.add_trace(mesh)
        # return 

    def add_trace_grid_img():
        """
        # 2次元グリッドの座標を定義
        # x = [-width/2 + d for d in range(int(width/2))]#[-1.5, 0, 1.5]
        # y = [-width/2 + d for d in range(int(width/2))]#[-1.5, 0, 1.5]
        x = [-width/2 + d for d in range(width+1)]#[-1.5, -0.5, 0.5, 1.5]
        y = [-height/2 + d for d in range(height+1)]#[-1.5, -0.5, 0.5, 1.5]
        # print(x, y)
        for i in range(width+1):
            for j in range(height+1):
                trace_gridx = go.Scatter3d(x=[x[j]]*(width+1), y=y, z=[camera[-1][-1] for _ in range(width+1)], mode='lines', line_width=2, line_color=color, name="pixel")
                fig.add_trace(trace_gridx)
                trace_gridy = go.Scatter3d(x=x, y=[y[i]]*(height+1), z=[camera[-1][-1] for _ in range(height+1)], mode='lines', line_width=2, line_color=color, name="pixel")
                fig.add_trace(trace_gridy)
        """

    # def add_traces_to_fig(self, cam, lines):
        # self.add_trace_cam_coord()
        # trace_screen = self.get_trace_screen(cam_pos, cam, lines)

        # data = trace_cam_coord# + trace_screen
        # data += trace_cube
        # for i in range(len(trace_ray_lis)):
        #     data += trace_ray_lis[i]
        # for j in range(len(trace_sampling_point_lis)):
        #     data += trace_sampling_point_lis[j]

        # self.fig.add_traces(data)
    

if __name__ == "__main__":
    # from logger_global import logger
    w_, h_ = 3, 3
    plot_scale = 1.0
    fov = 80#120#45
    cam_pos = np.array([0, 0, 0])
    cam_lookat = np.array([0, 0, -1])
    cam_up = np.array([0, 1, 0])
        
    cam_plotter = CameraPlotter(w_, h_, cam_pos, cam_lookat, cam_up, fov, plot_scale)

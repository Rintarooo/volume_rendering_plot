import plotly.graph_objects as go

class BasePlotter:
    def __init__(self):
        self.fig = go.Figure()
    
    def fig_show(self, plot_range = 15):
        self.fig.update_layout(
            scene =dict(
                xaxis = dict(range=(-plot_range, plot_range)),
                yaxis = dict(range=(-plot_range, plot_range)),
                zaxis = dict(range=(-plot_range, plot_range)),
                camera = dict(
                    up = dict(x=0,y=1,z=0),
                )
        ))
        self.fig.update_layout(
            scene=dict(
                aspectmode='cube'
            )
        )
        self.fig.update_layout(showlegend=False)
        self.fig.show()

    

import matplotlib.pyplot as mpp
from matplotlib._png import read_png

class Scene():
    
    def __init__(self, xtitle="-", ytitle="-"):
        self.fig = mpp.figure()
        self.fig.ax = self.fig.add_subplot(111) 
        self.fig.ax.set_xlabel(xtitle)
        self.fig.ax.set_ylabel(ytitle)
        self.has_data = False

    def add_series(self, xdata, ydata, label):
        self.fig.ax.plot(xdata, ydata, label = label)
        self.has_data = True
        
    def add_glyph(self, glyph):
        glyph.attach_to_axes(self.fig.ax)
        
    def add_image(self, filename):
        self.fig.ax.imshow(read_png(filename))
        
    def add_tiff(self, filename):
        self.fig.ax.imshow(mpp.imread(filename))
        
    def show(self):
        self.fig.ax.autoscale()
        if self.has_data:
            self.fig.ax.legend(loc=0)
        return self.fig
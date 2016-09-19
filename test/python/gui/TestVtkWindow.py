''' The chaste GUI app and main frame.

'''

import wx
import vtk
import chaste.gui.sub_panels.canvas3d
import chaste.geometry

class Frame(wx.Frame):
    
    def __init__(self, parent):
        
        '''
        Set up the main frame.
        
        Create managers for event handling and the main menus and panels.
        ''' 
        
        wx.Frame.__init__(self, parent, size = (800,600))

        self.create_panels()
        self.Show(True)
    
    def create_panels(self):
        
        ''' 
        Create and size the main panels
        ''' 
        
        self.panel = wx.Panel(self, wx.ID_ANY)
        self.panel.choice = chaste.gui.sub_panels.canvas3d.Panel(self.panel)
        
        #domain = chaste.geometry.Part()
        #domain.AddCuboid(100.0, 100.0, 1.0)
        
        #polyd = domain.GetVtk()
       # mapper = vtk.vtkPolyDataMapper()
        #mapper.SetInput(polyd)
        #mapper.ScalarVisibilityOn()
        #actor = vtk.vtkActor()
        #actor.SetMapper(mapper)
        
        #self.panel.choice.add_mesh(actor)

        hbox = wx.BoxSizer()
        hbox.Add(self.panel.choice, 1, wx.EXPAND, 1)
        self.panel.SetSizer(hbox) 
        
        wx.FutureCall(3, self.Destroy())
                
if __name__ == "__main__":

    # Launch the window
    app = wx.App(redirect=False)
    app.frame = Frame(parent=None)
    app.MainLoop()
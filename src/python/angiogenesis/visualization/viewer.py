import mpl_scene

class Mesh2d():
    
    def __init__(self, mesh, chaste_format = True, tri_format = False):
        self.mesh = mesh
        self.chaste_format = chaste_format
        self.tri_format = tri_format
        
    def show(self):
        
        self.scene = chaste_project_Angiogenesis.visualization.two.scene.Scene()
        
        if self.chaste_format:
            mesh_glyph = chaste_project_Angiogenesis.visualization.two.glyphs.MeshGlyph(self.mesh, self.chaste_format)
        elif self.tri_format:
            mesh_glyph = chaste_project_Angiogenesis.visualization.two.glyphs.MeshGlyph([self.mesh.points, self.mesh.elements], self.chaste_format)
        else:
            mesh_glyph = chaste_project_Angiogenesis.visualization.two.glyphs.MeshGlyph(self.mesh, self.chaste_format)
        self.scene.add_glyph(mesh_glyph)
        
        return self.scene.fig
    
class Image2d():
    
    def __init__(self, path):
        self.path = path
        
    def show(self):
        
        self.scene = mpl_scene.Scene()
        self.scene.add_tiff(self.path)
        
        return self.scene.fig   #
    
class PolyData2d():
    
    def __init__(self, polydata):
        self.polydata = polydata
        
        
    def show(self):
        
        self.scene = mpl_scene.Scene()
        
        converter = code.geometry.converters.VtkToTri(self.polydata)
        mesh = converter.generate()
        
        mesh_glyph = chaste_project_Angiogenesis.visualization.two.glyphs.MeshGlyph(mesh, False)

        self.scene.add_glyph(mesh_glyph)
        
        return self.scene.fig   
import open3d as o3d
import numpy as np
import laplacian_surface_editing_master.Proj_edge.library.Mesh

mesh = o3d.io.read_triangle_mesh("bunny.ply")

handlepoint= 10

handle={}
handle[handlepoint] = np.asarray(mesh.vertices)[handlepoint]
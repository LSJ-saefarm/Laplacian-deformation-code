# %% 
import open3d as o3d
import numpy as np

# %%
mesh = o3d.io.read_triangle_mesh("model_normalized.obj")
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], width=1000, height=800, mesh_show_back_face=True)

# %% Finding view matrix
def callBack(self):
    print(self.get_view_control().convert_to_pinhole_camera_parameters().extrinsic)

o3d.visualization.draw_geometries_with_key_callbacks([mesh], {ord("S"): callBack}, width=1000, height=800)

# %% View matrix to lookat vector
view_matrix = np.array([[ 1., 0., 0., -2.26212382],
                        [0., -1., 0., -2.5267005],
                        [0., 0., -1., 0.77461535],
                        [0., 0., 0., 1.]])
initial_lookat = np.array([0, 0, -1, 1])

lookat = np.matmul(initial_lookat, view_matrix)
lookat = np.array([lookat[0]/lookat[3], lookat[1]/lookat[3], lookat[2]/lookat[3]])
print(lookat)

# %% Contour vertices visualization
mesh.paint_uniform_color([0.8, 0.8, 0.8])
positive = 0
all = 0
idx = []
for i, normal in enumerate(mesh.vertex_normals):
    all += 1
    if np.dot(normal, lookat) > 0:
        positive += 1
        idx.append(i)
np.asarray(mesh.vertex_colors)[idx[0:], :] = [1.0, 0.0, 0.0]
o3d.visualization.draw_geometries([mesh], width=1000, height=800, mesh_show_back_face=True)

print(positive, all)

# %% extract all edges and triangle indices
mesh.paint_uniform_color([0.8, 0.8, 0.8])
edges = []
triangles = []
for i, indices in enumerate(mesh.triangles): # mesh_triangle 에서 indices (몇번째 vertice인지) 등등등
    for edge_indices in [[0, 1], [1, 2], [2, 0]]:
        v1 = indices[edge_indices[0]]
        v2 = indices[edge_indices[1]]
        if v1 < v2:
            edge = [v1, v2]
        else:
            edge = [v2, v1]
        # print("Edge: ", edge)
        found = False
        for j, e in enumerate(edges):
            if edge == e:
                triangles[j].append(i)
                found = True
        if found == False:
            edges.append(edge)
            triangles.append([i])
        # print("Edges: ", edges)
        # print("Triangles: ", triangles)
    # if i == 2:
    #     break

print(np.dot([1,0,0], [1,1,0]))

# %% Contour edges visualization
mesh.paint_uniform_color([0.8, 0.8, 0.8])
mesh_normals = np.asarray(mesh.triangle_normals)
inverse = 0
all = 0
idx = []

lines = []
for i, e in enumerate(edges): #초기 edges 정제 후 작업
    all += 1
    if len(triangles[i]) == 1: continue
    n1 = np.dot(mesh_normals[triangles[i][0]], lookat)
    n2 = np.dot(mesh_normals[triangles[i][1]], lookat)
    if (n1 <= 0 and n2 >= 0) or (n1 >= 0 and n2 <= 0):
        inverse += 1
        idx.append(e[0])
        idx.append(e[1])
        lines.append(e)
        # np.append(np.asarray(lineset.colors), [[0., 0., 1.]], axis=0)
        # np.append(np.asarray(lineset.lines), [edge], axis=0)


print(inverse, all)
line_colors = [[1.0, 0, 0] for i in range(len(lines))]
line_set = o3d.geometry.LineSet(
    points=mesh.vertices,
    lines=o3d.utility.Vector2iVector(lines),
)
print(lines)

#현재는 index 형식으로 표기되어있음. ->따라서 좌표 형식으로 바꾸는 것이 필요함.
#보는 각도 벡터가 있으면 대략적으로 변환 가능
#만들어둔 sketch.py를 사용해 3d 상에서 연속적인 contour를 찾으면 된다.
# 2d space -> 3d space를 하는데


line_set.colors = o3d.utility.Vector3dVector(line_colors)
o3d.visualization.draw_geometries([mesh, line_set], width=1000, height=800, mesh_show_back_face=True)
# %%
# o3d.io.write_line_set("09620_light_lineset.ply", line_set, write_ascii=False, compressed=False, print_progress=False)
# %% 
import open3d as o3d
import numpy as np



def edge_extraction(mesh, view):
    # %% Faster edge extraction

    # Calculate view direction
    view_matrix = np.array(view)
    initial_lookat = np.array([0, 0, -1, 1])
    lookat = np.matmul(initial_lookat, view_matrix)
    lookat = np.array([lookat[0] / lookat[3], lookat[1] / lookat[3], lookat[2] / lookat[3]])

    # Set mesh
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()
    vertices = np.asarray(mesh.vertices)
    # vertex_normals = np.asarray(mesh.vertex_normals)
    triangles = np.asarray(mesh.triangles)
    triangle_normals = np.asarray(mesh.triangle_normals)
    mesh = mesh.compute_adjacency_list()
    adj_list = mesh.adjacency_list

    # Find front-face and back-face
    triangle_direcs = np.full((len(triangles),), False)
    for i, n in enumerate(triangle_normals):
        dot = np.dot(n, lookat)
        if dot > 0:
            triangle_direcs[i] = True
    # print(triangle_direcs)

    # Find vert2face
    vert2face = [set([]) for i in range(len(vertices))]
    for ti, t in enumerate(triangles):
        for vi in t:
            vert2face[vi].add(ti)

    # Find contour edges
    idx = []
    lines = []
    for i, v in enumerate(vertices):
        for j in adj_list[i]:
            if j <= i: continue
            union = tuple(vert2face[i] & vert2face[j])
            if triangle_direcs[union[0]] ^ triangle_direcs[union[1]]:
                idx.append(i)
                idx.append(j)
                lines.append([i, j])

    return lines
    '''line_colors = [[1.0, 0, 0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet(
        points=mesh.vertices,
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(line_colors)
    o3d.visualization.draw_geometries([mesh, line_set], width=1000, height=800, mesh_show_back_face=True)'''
    '''
    mesh.compute_vertex_normals()
    view_matrix = np.array(view)
    initial_lookat = np.array([0, 0, -1, 1])

    lookat = np.matmul(initial_lookat, view_matrix)
    lookat = np.array([lookat[0] / lookat[3], lookat[1] / lookat[3], lookat[2] / lookat[3]])

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
    # o3d.visualization.draw_geometries([mesh], width=1000, height=800, mesh_show_back_face=True)

    # print(positive, all)

    edges = []
    triangles = []
    for i, indices in enumerate(mesh.triangles):  # mesh_triangle 에서 indices (몇번째 vertice인지) 등등등
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

    mesh_normals = np.asarray(mesh.triangle_normals)
    inverse = 0
    all = 0
    idx = []

    lines = []
    for i, e in enumerate(edges):  # 초기 edges 정제 후 작업
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

    return lines  # index들로 이루어진 line을 return함
    # 현재는 index 형식으로 표기되어있음. ->따라서 좌표 형식으로 바꾸는 것이 필요함.
    # 보는 각도 벡터가 있으면 대략적으로 변환 가능
    # 만들어둔 sketch.py를 사용해 3d 상에서 연속적인 contour를 찾으면 된다.
    # 2d space -> 3d space를 하는데

    # %%
    # o3d.io.write_line_set("09620_light_lineset.ply", line_set, write_ascii=False, compressed=False, print_progress=False)
# %%'''
if __name__=="__main__":
    mesh = o3d.io.read_triangle_mesh("model_normalized.obj")
    V= np.array([[ 1., 0., 0., -2.26212382],
                            [0., -1., 0., -2.5267005],
                            [0., 0., -1., 0.77461535],
                            [0., 0., 0., 1.]])
    print(edge_extraction(mesh, V)) #View matrix
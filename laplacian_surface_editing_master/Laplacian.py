import numpy as np
import networkx as nx
import trimesh
from Proj_edge.SketchP import *
from utils import *
import scipy.sparse
import time
import copy
#----------------hierarchy------------
'''Application
    -main을 call (input: mesh) output: 바뀐 mesh 
        -sketchP (input: mesh, viewM, draw_line), output: mesh 위의 patch
            -edge_extraction output: mesh의 edge (윤곽선)'''

def Laplac(mm, View):
    print(View['Detail'])
    mes=copy.deepcopy(mm)
    mes.compute_vertex_normals()
    mes.remove_unreferenced_vertices()
    mesh = trimesh.Trimesh(np.asarray(mes.vertices), np.asarray(mes.triangles),
                           vertex_normals=np.asarray(mes.vertex_normals))  #Trimesh로 변환예정

    print(len(mesh.vertices), len(mes.vertices))
    g = mesh_to_graph(mesh)


    M=easyMesh(mes)
    TempZip=Sketch2Patch(M, View)
    if TempZip==None:
        return None
    else:
        handle, _, _, Patch, _ = TempZip

    '''handles = {
        24092 : [-0.01134 ,  0.151374 , -0.0242688]  # 점.
    }
    boundary_ctrl_points = [15617, 24120, 30216, 11236, 6973] #boundary 점들
    '''
    TTT={}
    for hand in handle:
        TTT[hand[1]] = hand[0]
    handles = TTT
    '''mesh = trimesh.load('./meshes/00288_symmetric_2k.ply', process=False)'''
    export_fn = './exports/chair_%s.ply' % (time.strftime("%Y%m%d%H%M%S", time.localtime()))

    print("---------------Edge start--------")
    Edged=[]
    for Pointz in Patch:
        A = 0
        B = 0
        for i, connection in enumerate(M.connected[Pointz]):
            if connection==1:
                A+=1
                if i in Patch:
                    B+=1
                if i not in Patch:
                    print("thisis Edge")
                    Edged.append(Pointz)
        print(f"{A}/{B}")

    print("------------------Edge extracted")
    boundary_ctrl_points=list(set(Edged))
    print(handles, boundary_ctrl_points)

    #-------------------------------- Build Graph ------------------------------- #
    for point in boundary_ctrl_points:
        A=0
        for point2 in boundary_ctrl_points:
            if M.connected[point][point2] == 1 or M.connected[point2][point] == 1:
                A+=1
        print(A)

    '''orderedboundary=[]
    ttz = 0
    def Boundaryorder(cur):
        orderedboundary.append(cur)
        A = None
        for i in boundary_ctrl_points:
            if M.connected[i][cur] == 1 or M.connected[cur][i] == 1:
                if i ==orderedboundary[0] and len(orderedboundary)!=2:
                    print("routed")
                    break
                if i not in orderedboundary:
                    A = Boundaryorder(i)'''



    def Boundaryorder(cur, path):
        print(f"----{path}----")
        A = []
        for i in boundary_ctrl_points:
            if i not in path and (M.connected[i][cur] == 1 or M.connected[cur][i] == 1):
                A.append(Boundaryorder(i, path+[i]))

        if len(A)==0:
            return path
        else:
            Rank = sorted(A, key=lambda B: len(B))
            print(Rank)
            return Rank[-1]
    '''orderedboundary=[]'''
    '''ttz=0
    while 1:
        def Boundaryorder(cur, path):
            orderedboundary.append(cur)
            A=None
            for i in boundary_ctrl_points:
                if M.connected[i][cur]==1 or M.connected[cur][i]==1:
                    if i == boundary_ctrl_points[ttz] and len(path)!=2:
                        print("route")
                        print(path)
                        print(len(path), len(boundary_ctrl_points))
                        return path
                    elif i not in path:
                        A = Boundaryorder(i, path+[i])



            if A==None:
                return path
            return A

        orderedboundary= Boundaryorder(boundary_ctrl_points[ttz], [boundary_ctrl_points[ttz]])
        if len(boundary_ctrl_points)!=len(orderedboundary):
            ttz+=1

        print(boundary_ctrl_points, orderedboundary)
        print(len(boundary_ctrl_points), len(orderedboundary))'''

    orderedboundary=Boundaryorder(boundary_ctrl_points[0], [boundary_ctrl_points[0]])

    print(len(boundary_ctrl_points), len(orderedboundary))


    if len(boundary_ctrl_points) != len(orderedboundary):
        print("irrelevancy error")
        return None

    boundary_ctrl_points=orderedboundary




    g = mesh_to_graph(mesh)

    print("----------------------------좁은간격------------------------------")
    print(nx.shortest_path(g, boundary_ctrl_points[0], boundary_ctrl_points[1]))


    '''boundary = get_boundary(g, boundary_ctrl_points)'''

    boundary=boundary_ctrl_points  #test

    print("-----compare boundary---------")
    print(len(boundary), len(boundary_ctrl_points))

    editable_verts = get_editable_vertices(g, boundary, list(handles.keys())[int(len(handles)/2)])

    # --------------------- Subgraph of the Editiable Region --------------------- #
    subgraph = g.subgraph(boundary + editable_verts)

    print(f"---------{len(subgraph.nodes)}----------{len(Patch)}-----")
    g2l = {}
    for n in subgraph.nodes:
        g2l[n] = len(g2l)
    l2g = list(subgraph.nodes)


    def get_local_neighbor(subgraph, n, l2g, g2l):
        nb = []
        for i in subgraph.neighbors(l2g[n]):
            nb.append(g2l[i])
        return nb


    # -------------------------- Build the Linear System ------------------------- #
    L = rw_laplacian_matrix(subgraph).todense()
    V = np.matrix([subgraph.nodes[i]['pos'] for i in subgraph.nodes])
    Delta = L.dot(V)
    n = L.shape[0]

    LS = np.zeros([3*n, 3*n])
    LS[0*n:1*n, 0*n:1*n] = (-1) * L
    LS[1*n:2*n, 1*n:2*n] = (-1) * L
    LS[2*n:3*n, 2*n:3*n] = (-1) * L

    for i in range(n):
        nb_idx = get_local_neighbor(subgraph, i, l2g, g2l)
        ring = np.array([i] + nb_idx)
        V_ring = V[ring]
        n_ring = V_ring.shape[0]

        A = np.zeros([n_ring * 3, 7])
        for j in range(n_ring):
            A[j]          = [V_ring[j,0],           0 ,   V_ring[j,2], -V_ring[j,1], 1, 0, 0]
            A[j+n_ring]   = [V_ring[j,1], -V_ring[j,2],            0 ,  V_ring[j,0], 0, 1, 0]
            A[j+2*n_ring] = [V_ring[j,2],  V_ring[j,1], -V_ring[j, 0],           0 , 0, 0, 1]

        # Moore-Penrose Inversion
        A_pinv = np.linalg.pinv(A)
        s = A_pinv[0]
        h = A_pinv[1:4]
        t = A_pinv[4:7]


        T_delta = np.vstack([
             Delta[i,0]*s    - Delta[i,1]*h[2] + Delta[i,2]*h[1],
             Delta[i,0]*h[2] + Delta[i,1]*s    - Delta[i,2]*h[0],
            -Delta[i,0]*h[1] + Delta[i,1]*h[0] + Delta[i,2]*s   ,
        ])

        LS[i, np.hstack([ring, ring+n, ring+2*n])] += T_delta[0]
        LS[i+n, np.hstack([ring, ring+n, ring+2*n])] += T_delta[1]
        LS[i+2*n, np.hstack([ring, ring+n, ring+2*n])] += T_delta[2]


    # ------------------- Add Constraints to the Linear System ------------------- #
    constraint_coef = []
    constraint_b = []

    # Boundary constraints
    boundary_idx = [g2l[i] for i in boundary_ctrl_points]
    for idx in boundary_idx:
        constraint_coef.append(np.arange(3*n) == idx)
        constraint_coef.append(np.arange(3*n) == idx + n)
        constraint_coef.append(np.arange(3*n) == idx + 2*n)
        constraint_b.append(V[idx, 0])
        constraint_b.append(V[idx, 1])
        constraint_b.append(V[idx, 2])

    # Handle constraints
    for gid, pos in handles.items():
        idx = g2l[gid]
        constraint_coef.append(np.arange(3*n) == idx)
        constraint_coef.append(np.arange(3*n) == idx + n)
        constraint_coef.append(np.arange(3*n) == idx + 2*n)
        constraint_b.append(pos[0])
        constraint_b.append(pos[1])
        constraint_b.append(pos[2])

    constraint_coef = np.matrix(constraint_coef)
    constraint_b = np.array(constraint_b)


    # -------------------------- Solve the Linear System ------------------------- #
    A = np.vstack([LS, constraint_coef])
    b = np.hstack([np.zeros(3*n), constraint_b])
    spA = scipy.sparse.coo_matrix(A)

    V_prime = scipy.sparse.linalg.lsqr(spA, b)


    # -------------------------- Output the Edited Mesh -------------------------- #
    new_pnts = []
    for i in range(n):
        new_pnts.append([V_prime[0][i], V_prime[0][i+n], V_prime[0][i+2*n]])

    new_mesh = mesh.copy()
    for idx, pnt in enumerate(new_pnts):
        gid = l2g[idx]
        new_mesh.vertices[gid] = pnt

    new_mesh.export(export_fn)

    print(np.asarray(new_mesh.faces))
    '''final_mesh=o3d.geometry.TriangleMesh()
    final_mesh.vertices=o3d.utility.Vector3dVector(new_mesh.vertices)
    final_mesh.triangles=o3d.utility.Vector3dVector(new_mesh.faces)'''
    final_mesh=new_mesh.as_open3d
    final_mesh.compute_triangle_normals()
    final_mesh.compute_vertex_normals()
    return final_mesh

if __name__ == "__main__":
    # --------------------------------- Load Mesh -------------------------------- #
    mesh = trimesh.load('./meshes/bunny.ply', process=False)
    export_fn = './exports/bunny_%s.ply' % (time.strftime("%Y%m%d%H%M%S", time.localtime()))

    # ---------------------------- Editing Parameters ---------------------------- #
    mesh = o3d.io.read_triangle_mesh("./meshes/bunny.ply")
    Laplac(mesh, None, None)
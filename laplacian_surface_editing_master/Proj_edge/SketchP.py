from .library.Mesh import *
from .edge_extraction_func import *
'''
def get_x_and_y(event):
    global lasx, lasy
    lasx, lasy = event.x, event.y

Drawingline=[]
def draw_smth(event):
    global lasx, lasy
    global Drawingline
    global Givenline #Test용 Givenline
    canvas.create_line((lasx, lasy, event.x, event.y),
                      fill='red',
                      width=2)
    Drawingline.append(TDPoint((lasx, lasy)))
    lasx, lasy = event.x, event.y


    Dist, Line, Point = Givenline.PointfromLine(Drawingline[0])
    if Line !=None:
        canvas.create_line((Line[0].x, Line[0].y, Line[1].x, Line[1].y), fill='red', width=2)
    elif Line is None and Point!=None:
        canvas.create_line((Point.x, Point.y, Point.x, Point.y), fill='red', width=5)
    Dist, Line, Point = Givenline.PointfromLine(Drawingline[-1])
    if Line != None:
        canvas.create_line((Line[0].x, Line[0].y, Line[1].x, Line[1].y), fill='red', width=2)
    elif Line is None and Point != None:
        canvas.create_line((Point.x, Point.y, Point.x, Point.y), fill='red', width=5)


def draw_line(line):
    points=line.points
    for i in range(len(points)-1):
        canvas.create_line((points[i].x, points[i].y, points[i+1].x, points[i+1].y), fill='blue', width=2)'''


'''app = Tk()
app.geometry("400x400")
canvas = Canvas(app, bg='black')
canvas.pack(anchor='nw', fill='both', expand=1)
image = Image.open("img.png")
image = image.resize((400,400), Image.ANTIALIAS)
image = ImageTk.PhotoImage(image)
canvas.create_image(0,0, image=image, anchor='nw')
draw_line(Givenline)

canvas.bind("<Button-1>", get_x_and_y)
canvas.bind("<B1-Motion>", draw_smth)
# line: point들의 집합

app.mainloop()'''

#mesh = o3d.io.read_triangle_mesh("bunny.ply")
#mesh.compute_vertex_normals()
def Matching(easymesh, line, path, View):
    sample = 20
    split = [int(len(path[0])*i/sample) for i in range(1, sample)]
    poin=[]
    for spliting in split:
        poin.append(path[0][spliting])

    Widget=View['widget3d']

    def ClosestPointOnLine(a, b, p): #all 3: TDPoint

        ap = p - a
        ab = b - a
        APnp=np.asarray([ap.x, ap.y, ap.z])
        ABnp = np.asarray([ab.x, ab.y, ab.z])
        Anp = np.asarray([a.x, a.y, a.z])
        result = Anp + np.dot(APnp, ABnp) / np.dot(ABnp, ABnp) * ABnp
        print("-----ClosestPoint-----")
        print(p)
        print(a,b)
        print(result)
        return result

    Final=[]
    for i, point in enumerate(poin):
        TDpo=line.percentPoint(split[i]/len(path[0]))
        world1 = Widget.widget3d.scene.camera.unproject(TDpo.x, TDpo.y, 0.1, Widget.widget3d.frame.width,
                                                       Widget.widget3d.frame.height)
        world2 = Widget.widget3d.scene.camera.unproject(TDpo.x, TDpo.y, 0.2, Widget.widget3d.frame.width,
                                                       Widget.widget3d.frame.height)
        W1=SDPoint(world1)
        W2=SDPoint(world2)
        Oripo=SDPoint(easymesh.mesh.vertices[point])
        print(ClosestPointOnLine(W1, W2, Oripo))
        Final.append((ClosestPointOnLine(W1, W2, Oripo).tolist() , point))


    return Final


def Sketch2Patch(mesh, View):
    # %% Finding view matrix
    '''  P=None
    def callBack(self):
        global P
        print(self.get_view_control().convert_to_pinhole_camera_parameters().extrinsic)
        P=self.get_view_control().convert_to_pinhole_camera_parameters().extrinsic

    o3d.visualization.draw_geometries_with_key_callbacks([mesh], {ord("S"): callBack}, width=1000, height=800)'''
    M=mesh
    #--------------이 부분을 edges: 윤곽선(red) / Givenline: sketch로 바꾸면 됨.
    print("edgeextraction")
    edges = edge_extraction(M.mesh, View['view_matrix'])  # Viewmatrix '''
    print(View['sketch'])
    Givenline=makeline(View['sketch'])
    '''V=None
    edges=edge_extraction_func.edge_extraction(mesh, V) #Viewmatrix '''
    '''
    '''
    print("Making contour")
    #mesh(easymesh), sketch, edge(윤곽선), View(여러 정보
    Path=MakeContour(M, Givenline, edges, View)
    if Path==None:
        return None



    MatchF = Matching(M, Givenline, Path, View) #각 Path의 point(idx) 별로 Givenline의 특정 점/ 특정 3d 좌표와 matching해주면 됨.
    print(MatchF)
    print("Making patch")
    Azeus = MakePatch(M,Path, View['Detail'])


    print("out")
    '''
    handlepoint=Path[0][max(0, int(len(Path)/2))]
    handle={}
    
    
    Vert=np.asarray(M.mesh.vertices)
    Oripoint=Vert[handlepoint].tolist()
    handle[handlepoint] = Oripoint'''

    #--------Test 용

    #현재는 index 형식으로 표기되어있음. ->따라서 좌표 형식으로 바꾸는 것이 필요함.
    #보는 각도 벡터가 있으면 대략적으로 변환 가능
    #만들어둔 sketch.py를 사용해 3d 상에서 연속적인 contour를 찾으면 된다.
    # 2d space -> 3d space를 하는데
    mesh=M.mesh
    mesh.paint_uniform_color([0.8, 0.8, 0.8])
    linez = []
    for i in Azeus:  # Patch에 포함되는 line들
        for j in Azeus:
            if M.connected[i][j] == 1:
                linez.append([i, j])

    line_colors = [[1.0, 0, 0] for i in range(len(linez))]
    line_setz = o3d.geometry.LineSet(
        points=mesh.vertices,
        lines=o3d.utility.Vector2iVector(linez),
    )
    line_setz.colors = o3d.utility.Vector3dVector(line_colors)


    lines = []  # lines, 원본 contour의 line
    for i in range(len(Path[0]) - 1):
        lines.append([Path[0][i], Path[0][i + 1]])


    line_colors = [[0, 1.0, 0] for i in range(len(lines))]
    line_settt = o3d.geometry.LineSet(
        points=mesh.vertices,
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_settt.colors = o3d.utility.Vector3dVector(line_colors)


    line_colors = [[0, 0, 1.0] for i in range(len(edges))]
    line_set = o3d.geometry.LineSet(
        points=mesh.vertices,
        lines=o3d.utility.Vector2iVector(edges),
    )
    line_set.colors = o3d.utility.Vector3dVector(line_colors)

    '''linek = []  # lines for the edge
    for i in AzeusEdgepoint:
        for j in AzeusEdgepoint:
            if M.connected[i][j] == 1:
                linek.append([i, j])

    line_colors = [[0, 0, 0] for i in range(len(linek))]
    line_setzz = o3d.geometry.LineSet(
        points=mesh.vertices,
        lines=o3d.utility.Vector2iVector(linek),
    )
    line_setzz.colors = o3d.utility.Vector3dVector(line_colors)'''

    '''o3d.visualization.draw_geometries([mesh, line_set, line_setz, line_settt], width=1000, height=800,
                                          mesh_show_back_face=True)'''

    '''print("Saving")
    M.SaveDist()'''
    return MatchF, None, Path, Azeus, None

# 전체 배열 말고, 일부분만 저장해서 최적화하기

# 전체적으로 속도 늘리기
# 시각화 개선하기
#어플리케이션 가기
if __name__=="__main__":
    mesh = o3d.io.read_triangle_mesh("bunny.ply")
    # mesh.compute_vertex_normals()
    # %% Finding view matrix
    P = None
    def callBack(self):
        global P
        print(self.get_view_control().convert_to_pinhole_camera_parameters().extrinsic)
        P = self.get_view_control().convert_to_pinhole_camera_parameters().extrinsic

    o3d.visualization.draw_geometries_with_key_callbacks([mesh], {ord("S"): callBack}, width=1000, height=800)
    Givenline = makeline([(0, 0), (100, 200), (200, 250), (350, 400)])
    print("meshmaking")
    M=easyMesh(mesh)

    print("call")

    View = {'sketch': self.lineset, '3dsketch': None, 'view_matrix': view_matrix,
            'direction': direc}  # 각각 원본 sketch(list),

    # 공중에 떠있는 sketch(o3d.lineset), view_matrix(numpy), direc(최종 camera direction), numpy
    _ , _, Path, Azeus, AzeusEdgepoint=Sketch2Patch(M, View)  #P: 4x4
    mesh.paint_uniform_color([0.8, 0.8, 0.8])
    linez = []
    for i in Azeus:  # Patch에 포함되는 line들
        for j in Azeus:
            if M.connected[i][j] == 1:
                linez.append([i, j])

    line_colors = [[1.0, 1.0, 1.0] for i in range(len(linez))]
    line_setz = o3d.geometry.LineSet(
        points=mesh.vertices,
        lines=o3d.utility.Vector2iVector(linez),
    )
    line_setz.colors = o3d.utility.Vector3dVector(line_colors)
    lines = []  # lines, 원본 contour의 line
    for i in range(len(Path[0]) - 1):
        lines.append([Path[0][i], Path[0][i + 1]])

    line_colors = [[0, 1.0, 0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet(
        points=mesh.vertices,
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(line_colors)
    linek = []  # lines for the edge
    for i in AzeusEdgepoint:
        for j in AzeusEdgepoint:
            if M.connected[i][j] == 1:
                linek.append([i, j])

    line_colors = [[1.0, 0, 0] for i in range(len(linek))]
    line_setzz = o3d.geometry.LineSet(
        points=mesh.vertices,
        lines=o3d.utility.Vector2iVector(linek),
    )
    line_setzz.colors = o3d.utility.Vector3dVector(line_colors)
    '''o3d.visualization.draw_geometries([mesh, line_set, line_setz, line_setzz], width=1000, height=800,
                                      mesh_show_back_face=True)'''
    # ----------------------------



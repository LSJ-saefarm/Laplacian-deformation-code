from laplacian_surface_editing_master.Proj_edge.library.Mesh import *

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

mesh = o3d.io.read_triangle_mesh("bunny.ply")
#mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], width=1000, height=800, mesh_show_back_face=True)

# %% Finding view matrix
P=None
def callBack(self):
    global P
    print(self.get_view_control().convert_to_pinhole_camera_parameters().extrinsic)
    P=self.get_view_control().convert_to_pinhole_camera_parameters().extrinsic

o3d.visualization.draw_geometries_with_key_callbacks([mesh], {ord("S"): callBack}, width=1000, height=800)

Tem=np.asarray({})
TemC=np.asarray([])
try:
    Tem=np.load('library/saved_distance.npy', allow_pickle='TRUE')
    print("savedDist")
except:
    Tem=np.asarray({})

try:
    TemC=np.load('library/saved_connection.npy', allow_pickle='TRUE')
    print("savedConnect")
    print(TemC)
    print(type(TemC))
except:
    TemC=np.asarray([])

M=easyMesh(mesh, Tem, TemC)

for j in [10219, 8201, 10106, 10031, 9206, 8470, 10849, 8651, 10624, 214, 6909, 14827]:
    print("for " +str(j))
    for i in [7, 3073, 3072, 1293, 1527, 3071, 183]:
        print(M.Blockdistance(j, i))


print("111")

#--------------이 부분을 edges: 윤곽선(red) / Givenline: sketch로 바꾸면 됨.
edges=[[7, 57],[57, 3073], [3073, 3072], [3072, 1293], [1293,1527],[1527, 3071],[3071, 183]]
Givenline=makeline([(0,0), (100,200),(200,250), (350,400)])

'''V=None
edges=edge_extraction_func.edge_extraction(mesh, V) #Viewmatrix '''
'''
'''

Path=MakeContour(M, Givenline, edges, P)
print(Path)
print("222")
Azeus, AzeusEdgepoint =MakePatch(M,Path)
print("333")
mesh.paint_uniform_color([0.8, 0.8, 0.8])


linez=[]
for i in Azeus:  #Patch에 포함되는 line들
    for j in Azeus:
        if M.connected[i][j]==1:
            linez.append([i,j])

line_colors = [[1.0, 1.0, 1.0] for i in range(len(linez))]
line_setz = o3d.geometry.LineSet(
    points=mesh.vertices,
    lines=o3d.utility.Vector2iVector(linez),
)
line_setz.colors = o3d.utility.Vector3dVector(line_colors)

lines=[] #lines, 원본 contour의 line
for i in range(len(Path[0])-1):
    lines.append([Path[0][i], Path[0][i+1]])

line_colors = [[0, 1.0, 0] for i in range(len(lines))]
line_set = o3d.geometry.LineSet(
    points=mesh.vertices,
    lines=o3d.utility.Vector2iVector(lines),
)
line_set.colors = o3d.utility.Vector3dVector(line_colors)


linek=[] #lines for the edge
for i in AzeusEdgepoint:
    for j in AzeusEdgepoint:
        if M.connected[i][j]==1:
            linek.append([i,j])


line_colors = [[1.0, 0, 0] for i in range(len(linek))]
line_setzz = o3d.geometry.LineSet(
    points=mesh.vertices,
    lines=o3d.utility.Vector2iVector(linek),
)
line_setzz.colors = o3d.utility.Vector3dVector(line_colors)

#----------------------------

handlepoint=lines[int(len(lines)/2)][0]
print(handlepoint)
handle={}
Vert=np.asarray(M.mesh.vertices)
handle[handlepoint] = Vert[handlepoint]
boundarycontrol = AzeusEdgepoint
print(np.asarray(mesh.vertices)[10])
#--------Test 용

#현재는 index 형식으로 표기되어있음. ->따라서 좌표 형식으로 바꾸는 것이 필요함.
#보는 각도 벡터가 있으면 대략적으로 변환 가능
#만들어둔 sketch.py를 사용해 3d 상에서 연속적인 contour를 찾으면 된다.
# 2d space -> 3d space를 하는데

o3d.visualization.draw_geometries([mesh, line_set, line_setz, line_setzz], width=1000, height=800, mesh_show_back_face=True)

A=[]
for a, i in enumerate(M.connected):
    for b, j in enumerate(i):
        if j==1:
            A.append((a,b))
np.save('saved_connection', A)
np.save('saved_distance', M.distanceBet)
# 전체 배열 말고, 일부분만 저장해서 최적화하기

# 전체적으로 속도 늘리기
# 시각화 개선하기
#어플리케이션 가기



import math
import numpy as np
import open3d as o3d
from .Mat import *

class easyMesh:
    def __init__(self, mesh):
        self.mesh=mesh
        self.PointNum=len(mesh.vertices)
        self.connected=[[0 for i in range(self.PointNum)] for j in range(self.PointNum)]
        Tem = np.asarray({})
        TemC = np.asarray([])
        '''try:
            Tem = np.load('saved_distance.npy', allow_pickle='TRUE')
            print("savedDist")
        except:
            Tem = np.asarray({})
        try:
            TemC = np.load('saved_connection.npy', allow_pickle='TRUE')
            print("savedConnect")
            print(TemC)
            print(type(TemC))
        except:
            TemC = np.asarray([])'''

        if len(TemC)>0: #Temc: list
            for Point in TemC:
                self.connected[Point[0]][Point[1]] = 1
                self.connected[Point[1]][Point[0]] = 1
        else:
            print("makingupconnection")
            for Triangle in mesh.triangles:  # connectivity는 항상 유지됨 (2d space로 옮겨도)
                for j in [[0, 1], [0, 2], [1, 2]]:
                    self.connected[Triangle[j[0]]][Triangle[j[1]]] = 1
                    self.connected[Triangle[j[1]]][Triangle[j[0]]] = 1

        self.Rededges=None
        if len(Tem.item())>0:
            self.distanceBet=Tem.item()
        else:
            self.distanceBet={}
        '''Tem=Tem.item()
        for te in Tem.keys(): #연결된 정보 파악
            self.distanceBet[te[0]][te[1]]=Tem[te]'''
        #print(np.asarray(mesh.triangles[:30]))
        #print(len(mesh.triangles))
        Tcount=0
    def SaveDist(self):
        return True


        A = []
        for a, i in enumerate(self.connected):
            for b, j in enumerate(i):
                if j == 1:
                    A.append((a, b))
        np.save('saved_connection', A)
        np.save('saved_distance', self.distanceBet)
        return True

    def Blockdistance(self, idxA, idxB):
        if (idxA, idxB) in self.distanceBet.keys() or (idxB, idxA) in self.distanceBet.keys():
            return self.distanceBet[(idxA, idxB)]
        if idxA==idxB:
            return 0
        if (idxA >=0 and idxA <self.PointNum and idxB>=0 and idxB < self.PointNum) is False :
            print("index error")
            return None
        visited=[idxA]
        visitedStep=[0]
        Cur=0
        while (1):
            #print(visited[Cur])
            #print(self.connected[visited[Cur]])
            if len(visited) <= Cur:
                print("No answer")
                return None

            for i, Next in enumerate(self.connected[visited[Cur]]): #i : index
                if Next == 1:
                    if i not in visited:
                        if i == idxB:
                            self.distanceBet[(idxA , idxB)] = visitedStep[Cur] + 1
                            self.distanceBet[(idxB , idxA)] = visitedStep[Cur] + 1
                            return visitedStep[Cur]+1
                        '''elif (i, idxB) in self.distanceBet.keys() or (idxB, i) in self.distanceBet.keys():
                            self.distanceBet[(idxA, idxB)] = visitedStep[Cur] + 1 + self.distanceBet[(i, idxB)]
                            self.distanceBet[(idxB, idxA)] = visitedStep[Cur] + 1 + self.distanceBet[(i, idxB)]
                            return visitedStep[Cur] + self.distanceBet[(i, idxB)] 틀린코드
                        '''
                        visited.append(i)
                        self.distanceBet[(idxA, i)] = visitedStep[Cur] + 1
                        self.distanceBet[(i, idxA)] = visitedStep[Cur] + 1
                        visitedStep.append(visitedStep[Cur]+1)
            Cur+=1

    def coord(self, idx):
        return self.mesh.vertices[idx]

    def TDmaker(self, View, edges, Ori): #P: camera matrix, edges: [e1, e2...], Ori: 기준점(원점)
        TDSD=[] #원본 3dpoint의 index 정보와, 변환된 좌표를 포함하고 있음.
        connected = []
        Standard=30  #--------수정요망
        for e in edges:
            TD1=SD2TD(View, SDPoint(self.mesh.vertices[e[0]]))
            TD2=SD2TD(View, SDPoint(self.mesh.vertices[e[1]]))
            '''
            if Euclid(TD1, Ori) > Standard and Euclid(TD2, Ori) > Standard:
                continue'''

            TDSD.append(((e[0], TD1), (e[1], TD2))) ##각각 e를 이루는 point의 index
            connected.append((e[0],e[1]))
            connected.append((e[1],e[0]))
        return TDSD, set(connected) #TDSD: TDpoint 정보 포함중
    def Pathmaker(self, edges):
        pass
    def clean(self):
        self. distanceBet={}
    def Test(self):
        test=[[0 for i in range(self.PointNum)] for j in range(self.PointNum)]
        for Triangle in self.mesh.triangles:  # connectivity는 항상 유지됨 (2d space로 옮겨도)
            for j in [[0, 1], [0, 2], [1, 2]]:
                test[Triangle[j[0]]][Triangle[j[1]]] += 1
                test[Triangle[j[1]]][Triangle[j[0]]] += 1
        for i in test:
            for j in test:
                if j!=2:
                    print("error")


def Fig(point, SDedges): #가장 가까운 SDedges의 argument를 반환, idx를 반환.
    low=1000000000
    lowData=None
    LL=None
    for Edge in SDedges:
        for Epoint in Edge:

            Dist=Euclid(point, Epoint[1])
            if Dist < low:
                lowData=Epoint[0]
                low=Dist
                LL=(point, Epoint[1])

    return lowData


def MakeContour(mesh, line, edges, View): #mesh: 원본 mesh(탐색용), line: 2d sketch, edges: 원본 mesh의 테두리 edge들 정보(idx)
    '''for e in edges:
        if mesh.connected[e[0]][e[1]]==0: #연결이 안된 edge가 존재하면
            print("EdgeError")
            break'''

    Original=(line.points[0]+line.points[-1])*(1/2)

    print(f"original points: {line.points[0]}, {line.points[-1]}")

    SDedges, EdgeConnection = mesh.TDmaker(View, edges, Original)  # edge를 3d/2d 정보를 포함하는 class로 만드는 함수list of 윤곽선 edge, 2d 좌표까지 함께있는 버전
    print(f"{len(SDedges)} out of {len(edges)}")
    idxA = Fig(line.points[0], SDedges)  # point에서 SDedges에서 가장 가까운 출력.
    idxB = Fig(line.points[-1], SDedges)
    print("최종 가장 가까운 점 ")
    print(idxA, idxB)
    print(f"Meshdistance: {mesh.Blockdistance(idxA, idxB) }")

    if mesh.Blockdistance(idxA, idxB)>= 30:
        print("Too long")
        return None

    Distoe = mesh.Blockdistance(idxA, idxB)
    def wholePath(idxo, idxs, idxe, path):
        #print(path)
        if idxs==idxe:
            return [([idxs], 0)]
        Whole=[]
        Distse = mesh.Blockdistance(idxe, idxs)
        Distos = mesh.Blockdistance(idxo, idxs)
        offs=max(0, 4-int(Distoe/7))
        for i, connect in enumerate(mesh.connected[idxs]): # i: 새로 가는 idx.
            '''print(idxs)
            print(sum(originalConnection[idxs]))'''
            if connect==1 and i not in path:
                isEdge=int(((i, idxs) in EdgeConnection))
                #dynamic 사용해서 실행 빠르게 하자
                Distie=mesh.Blockdistance(idxe, i)
                Distoi=mesh.Blockdistance(idxo, i)
                if (Distie < Distse and Distos < Distoi and Distoi+Distie <= Distoe + offs):
                    for Path in wholePath(idxo, i, idxe, path+[i]):
                        Whole.append(([idxs]+Path[0], Path[1]+isEdge))
        return Whole

    A=wholePath(idxA, idxA, idxB, [idxA])
    Rank=sorted(A, key=lambda A: A[1])
    print("-------pathfound---------")
    print(Rank)
    return Rank[-1]
    # 각각 처음, 끝 point에서 가장 가까운 edge를 찾은 뒤, 그 사이를 연결하는 edge들을 찾으면 된다.

def MakePatch(mesh, path, detail):

    road = path[0]
    Patch=[]
    offset = 2
    PatchEdgePoint = []

    criteria = detail


    for j in road:
        idxA = j
        Patch.append(idxA)
        visited = [idxA]
        visitedStep = [0]
        Temzz = [0]
        Cur = 0
        while 1:
            if visitedStep[Cur] > criteria:
                break
            for i, Next in enumerate(mesh.connected[visited[Cur]]):  # i : index
                if Next == 1:
                    if i not in visited:
                        Patch.append(i)
                        visited.append(i)
                        visitedStep.append(visitedStep[Cur] + 1)
            Cur +=1
        '''Max=max(Temzz)
        for tem in Temzz:
            if tem==Max:'''

    ''' for i in range(mesh.PointNum):
        for j in path[0]:
            if mesh.Blockdistance(i,j)<criteria:
                patch.append(i)
                break'''
    #global Vis # 방문했던 곳들..\
    #return Vis

    '''print(max(visitedStep), visitedStep, visited)
    PatchEdgePoint=[A for i, A in enumerate(visited) if (A in Patch) and (visitedStep[i] == max(visitedStep))]'''
    ''' print(len(Patch))
    print(len(PatchEdgePoint))
    print(PatchEdgePoint)
    for point in PatchEdgePoint:
        A = 0
        for point2 in PatchEdgePoint:
            if mesh.connected[point][point2] == 1 or mesh.connected[point2][point] == 1:
                A += 1
        print(A)'''

    print("-----------------------------------------------------------------------------")
    Patch=set(Patch)
    return set(Patch)



def SD2TD(View, SDpoint): #3Dpoint ->2Dpoint를 camera에 맞게끔 보여준다.
    '''X=np.asarray([[SDpoint.x], [SDpoint.y], [SDpoint.z],[1]])
    P=np.asarray(View['view_matrix'])
    res=P@X'''
    '''    print(res) # 여기서 차원을 맞춰야됨.
    print(res.reshape(1,4).tolist())'''
    '''if event.key == ctrl_key and event.type == gui.KeyEvent.Type.UP:
                        view_mtx = self.widget3d.scene.camera.get_view_matrix()
                        proj_mtx = self.widget3d.scene.camera.get_projection_matrix()
                        coord3d = [0, 0, 0, 1]
                        coord = np.matmul(proj_mtx, np.matmul(view_mtx, coord3d))
                        x = ((coord[0] / coord[3]) / 2.0 + 0.5) * self.widget3d.frame.width
                        y = ((coord[1] / coord[3]) / 2.0 + 0.5) * self.widget3d.frame.height
                        print(x, y)
                        return gui.Widget.EventCallbackResult.HANDLED'''
    view_mtx = View['view_matrix']
    proj_mtx = View['proj_matrix']
    coord3d = [SDpoint.x, SDpoint.y, SDpoint.z, 1]
    coord = np.matmul(proj_mtx, np.matmul(view_mtx, coord3d))
    x = ((coord[0] / coord[3]) / 2.0 + 0.5) * View['framesize'][0]
    y = 800-((coord[1] / coord[3]) / 2.0 + 0.5) * View['framesize'][1]

    return TDPoint((x,y))

def TD2SD(View, TD): #3Dpoint ->2Dpoint를 camera에 맞게끔 보여준다.
    TDc=np.asarray([[TD.x], [TD.y]])
    ViewM=np.asarray([[1/2*View['framesize'][0], 0, 0, 0.5*View['framesize'][0]], [0, 1/2*View['framesize'][1], 0, 0.5*View['framesize'][1]]])
    view_mtx = View['view_matrix']
    proj_mtx = View['proj_matrix']

    coord2d = np.asarray([[TD.x], [800-TD.y]])

    A=np.matmul(np.linalg.inv(ViewM), coord2d)
    B=np.matmul(np.matmul(np.linalg.inv(proj_mtx), A))
    C=np.matmul(np.linalg.inv(view_mtx), B)

    result= np.matmul(np.linalg.inv(view_mtx), np.matmul(np.linalg.inv(proj_mtx), np.matmul(np.linalg.inv(ViewM), coord2d)))
    print(result)
    '''coordO = [(TD.x/View['framesize'][0]-0.5) *2.0 * 1, (800-TD.y/View['framesize'][1]-0.5) *2.0 * 1, 0, 1]
    coord = np.matmul(np.linalg.inv(view_mtx), np.matmul(np.linalg.inv(proj_mtx), coordO))

    x = ((coord[0] / coord[3]) / 2.0 + 0.5) * View['framesize'][0]
    y = 800-((coord[1] / coord[3]) / 2.0 + 0.5) * View['framesize'][1]
    x, y, z= [1,2,3]'''
    return SDPoint((result[0]/result[3],y,z))

# 하고싶은것: 선을 쫙 그으면 관련 contour가 3d, 2d 상에서 부드럽게 이어지는 것.
#구현을 위해서는, sketch와

#---------------------------앱 구동

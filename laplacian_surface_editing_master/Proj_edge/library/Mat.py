import math
import numpy as np
import open3d as o3d

def Euclid(a, b):
    return(math.sqrt((a.x-b.x)**2+(a.y-b.y)**2))

class TDPoint: #해당 Camera view에서 보는 평면에서의 좌표, sketch
    def __init__(self, TD):
        self.x, self.y = TD
    def __str__(self):
        return str(tuple([self.x, self.y]))
    def __add__(self, other):
        return TDPoint((self.x + other.x , self.y + other.y))
    def __sub__(self, other):
        return TDPoint((self.x - other.x , self.y - other.y))
    def __mul__(self, num):
        return TDPoint((self.x*num, self.y*num))

class SDPoint: #절대 좌표의 위치
    def __init__(self, SD):
        self.x, self.y, self.z=SD
    def __str__(self):
        return str(tuple([self.x, self.y, self.z]))
    def __add__(self, other):
        return SDPoint((self.x + other.x , self.y + other.y, self.z + other.z))
    def __sub__(self, other):
        return SDPoint((self.x - other.x , self.y - other.y, self.z - other.z))
    def __mul__(self, num):
        return SDPoint((self.x*num, self.y*num, self.z*num))


def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))
'''
def angle(v1, v2):
  try:
    threshold=0.97
    if length(v1)==0 or length(v2)==0:
        return math.pi/2
    elif dotproduct(v1, v2) / (length(v1) * length(v2)) >= threshold:
        return 0
    else:
        return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))
  except:
    print(v1, v2)
    return None'''
def angle(v1, v2):
    import numpy as np
    v1 = np.array(v1)
    v2 = np.array(v2)
    angle = np.math.atan2(np.linalg.det([v1, v2]), np.dot(v1, v2))
    return abs(angle)

def middle(pointA, pointB, mid, sample):
    return TDPoint((pointA.x+ (pointB.x-pointA.x)*mid/sample, pointA.y+ (pointB.y-pointA.y)*mid/sample))


def makeline(points): # input: list of tuple(2)
    sample=30
    linepoints=[]
    for i in range(len(points)-1):
        for j in range(sample):
            linepoints.append(middle(TDPoint((points[i][0], points[i][1])), TDPoint((points[i+1][0], points[i+1][1])), j, sample))
    return line(linepoints)

#------이하 sketch와 x, y

class line:  #2d space에서의 일련의 point들.
    def __init__(self, points): #points: 순서대로 배치되어있음.
        self.points=points
        dist=[]
        for i in range(len(points)-1) :
            dist.append(Euclid(points[i], points[i+1]))
        self.dist=dist
        self.wholeDist=sum(dist)
    def PointfromLine(self, point):  #특정 point에서 line까지의 거리와 어떤 점 or edge과 가장 가까운지 등등을 출력, 현재 윤곽선 과정은 이걸 사용중. 개선 필요할듯(
        LastLine=None
        LastPoint=None
        LowDist=100000000
        for i in range(len(self.points)-1):
            #Vector인데 point로 표현
            lineV=[(self.points[i+1].x-self.points[i].x), (self.points[i+1].y-self.points[i].y)]
            fromV1=[(point.x-self.points[i].x), (point.y-self.points[i].y)]
            fromV2 = [(point.x - self.points[i+1].x), (point.y - self.points[i+1].y)]
            if angle(lineV, fromV1) <= math.pi/2 and angle(lineV, fromV2) >=math.pi/2: #적절한 line을 찾아냈을 경우
                Dist=length(fromV1)*math.sin(angle(lineV, fromV1))
                if Dist<LowDist:
                    LowDist=Dist
                    LastLine=[self.points[i], self.points[i+1]]
                    if LastPoint != None:
                        LastPoint = None
            if length(fromV1)<LowDist:
                LowDist=length(fromV1)
                LastPoint=self.points[i]
                if LastLine !=None:
                    LastLine=None
            if length(fromV2) < LowDist:
                LowDist = length(fromV2)
                LastPoint = self.points[i+1]
                if LastLine !=None:
                    LastLine=None
        return (LowDist, LastLine, LastPoint)
    def percentPoint(self, Percent):
        A=0

        for i, dist in enumerate(self.dist):
            if A <= Percent*self.wholeDist and A+dist >= Percent*self.wholeDist:
              tempp= A/self.wholeDist
              zz = Percent-tempp
              A= dist/self.wholeDist
              if zz==0:
                  Final=0
              else:
                Final= zz/A
              return middle(self.points[i], self.points[i+1], Final, 1)
            A=A+dist

if __name__=="__main__":
    A= TDPoint((1,2))
    B= TDPoint((2,3))
    print(A)
    Y=(A+B)*(1/2)
    print(Y.x)
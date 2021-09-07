import numpy as np

class OBB:
    def __init__(self,point_set:np.ndarray):
        self.pos=(point_set[0]+point_set[6])/2 #包围盒中心点位置
        self.axisX=self.__norm(point_set[1]-point_set[0]) #包围盒x轴的方向向量
        self.axisY=self.__norm(point_set[3]-point_set[0]) #包围盒y轴的方向向量
        self.axisZ=self.__norm(point_set[0]-point_set[4]) #包围盒z轴的方向向量
        self.half_size=np.array([self.__get_distance(point_set[0],point_set[1])/2,
                                 self.__get_distance(point_set[0],point_set[3])/2,
                                 self.__get_distance(point_set[0],point_set[4])/2])

    def __norm(self,vector): #将向量归一化为标准向量
        s=0
        for e in vector:
            s+=e*e
        return vector/(s**0.5)

    def __get_distance(self,point_1,point_2): #计算两个点的距离
        return ((point_1[0]-point_2[0])**2+(point_1[1]-point_2[1])**2+(point_1[2]-point_2[2])**2)**0.5

def cross_product(vector1,vector2): #向量积
    return np.array([vector1[1]*vector2[2]-vector1[2]*vector2[1],vector1[2]*vector2[0]-vector1[0]*vector2[2],vector1[0]*vector2[1]-vector1[1]*vector2[0]])

def getSeparatingPlane(r_pos,plane,box1:OBB,box2:OBB): #判断在选定的坐标平面是否有分割平面
    return ((abs(sum(r_pos*plane)) >
        (abs(sum((box1.axisX*box1.half_size[0])*plane)) +
        abs(sum((box1.axisY*box1.half_size[1])*plane)) +
        abs(sum((box1.axisZ*box1.half_size[2])*plane)) +
        abs(sum((box2.axisX*box2.half_size[0])*plane)) +
        abs(sum((box2.axisY*box2.half_size[1])*plane)) +
        abs(sum((box2.axisZ*box2.half_size[2])*plane)))))

def isCollision(box1:OBB,box2:OBB): #判断两个OBB是否发生碰撞
    r_pos=box2.pos-box1.pos

    if not (getSeparatingPlane(r_pos, box1.axisX, box1, box2) or getSeparatingPlane(r_pos, box1.axisY, box1, box2) or
            getSeparatingPlane(r_pos, box1.axisZ, box1, box2) or getSeparatingPlane(r_pos, box2.axisX, box1, box2) or
            getSeparatingPlane(r_pos, box2.axisY, box1, box2) or getSeparatingPlane(r_pos, box2.axisZ, box1, box2) or
            getSeparatingPlane(r_pos, cross_product(box1.axisX,box2.axisX), box1, box2) or
            getSeparatingPlane(r_pos, cross_product(box1.axisX,box2.axisY), box1, box2) or
            getSeparatingPlane(r_pos, cross_product(box1.axisX,box2.axisZ), box1, box2) or
            getSeparatingPlane(r_pos, cross_product(box1.axisY,box2.axisX), box1, box2) or
            getSeparatingPlane(r_pos, cross_product(box1.axisY,box2.axisY), box1, box2) or
            getSeparatingPlane(r_pos, cross_product(box1.axisY,box2.axisZ), box1, box2) or
            getSeparatingPlane(r_pos, cross_product(box1.axisZ,box2.axisX), box1, box2) or
            getSeparatingPlane(r_pos, cross_product(box1.axisZ,box2.axisY), box1, box2) or
            getSeparatingPlane(r_pos, cross_product(box1.axisZ,box2.axisZ), box1, box2)):
        return True
    else:
        return False

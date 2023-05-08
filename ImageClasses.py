import numpy as np
#Class file that supports the ray tracer

#Represents a sphere in the space
class sphere:
    
    def __init__(self,pos,scale,colour,ks,n,name):
        self.pos = pos
        self.scale = scale
        self.colour = colour
        self.n = n
        self.id = name
        self.ks = ks
#Builds the inverse transform for the sphere       
    def undoTransforms(self):
        
        M = np.array([[1/self.scale[0],0,0,0],
                      [0,1/self.scale[1],0,0],
                      [0,0,1/self.scale[2],0],
                      [0,0,0,1]])

        M2 = np.array([[1,0,0,-self.pos[0]],
                      [0,1,0,-self.pos[1]],
                      [0,0,1,-self.pos[2]],
                      [0,0,0,1]
                       ])

        self.InverseT = M2
        self.InverseS = M
    


    def print(self):
        
        print("Sphere Name = " + self.id)
        print("Pos = ")
        print(self.pos)
        print("Scale = ")
        print(self.scale)
        print("Colour = ")
        print(self.colour)
        print("K values = ")
        print(self.ks)
        print("N")
        print(self.n)


#Contains the details about the scene and image plane
class image:
    def __init__(self,plane,res,light,backgroundC,backgroundA):
        self.plane = plane
        self.res = res
        self.lights = light
        self.backgroundC = backgroundC
        self.backgroundA = backgroundA
        self.eye = [0,0,0]

#Builds a ray from the two pixel points, and shoots it into the scene
class startRay:
    def __init__(self,r,c,image):
        self.point = np.array([image.eye[0],image.eye[1],image.eye[2],1])

        for el in image.plane:
            if el[0] == "NEAR":
                z = -el[1]
            if el[0] == "RIGHT":
                x = el[1]*(2*c/image.res[0] - 1)
            if el[0] == "TOP":
                y = el[1]*(2*r/image.res[1] - 1)
        self.line = np.array([x,y,z,0])

#Im pretty sure this is useless but when I delete it the program complains
    def transform(self,o,c):
        self.point = [o[0],o[1],o[2]]
        self.line = [c[0],c[1],c[2]]

#Converts the Ray into sphere space
    def inverseTransform(self,sphere):
        a = (sphere.InverseT.dot(self.point).dot(sphere.InverseS))
        b = (sphere.InverseT.dot(self.line).dot(sphere.InverseS))
        

        self.inverseP = [a[0],a[1],a[2]]
        self.inverseL = [b[0],b[1],b[2]]

    
#Class for shadow rays (or any ray with line L, and starting from point P)
class shadowRay(startRay):

    def __init__(self,point,line):
        self.point = np.array(point)
        self.line = np.array([line[0],line[1],line[2],0])
        #print(self.point)
        #print(self.line)
        self.inverseL = False
        self.inverseP = False

#Pretty much the same as shadow ray, but has an inverse transpose function
class normal(startRay):

    def __init__(self,point,line):
        #print(point)
        self.point = np.array(point)
        self.line = np.array(line)

    def inverseTranspose(self,M1,M2):

        M1 = M1.transpose() 
        M2 = M2.transpose()
        #print(M2.dot(M1))
        self.line = M2.dot(M1).dot(self.line)
        self.point = M2.dot(M1).dot(self.point)

    
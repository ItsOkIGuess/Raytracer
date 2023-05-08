#This program uses these libraries
try:
    import ImageClasses
    import sys
    import numpy as np
    import math
except:
    print("This program uses the libraries: sys, numpy, and math")
    print("please run:")
    print("pip install numpy")
    print("pip install math")
    print("pip install sys")

#Calculates the length of a vector
def vecLength(a):
    sum = 0
    for el in a:
        sum = sum + el**2

    return(math.sqrt(sum))

#Subtracts two vectors (a-b)
def vecSubtraction(a,b):
    c = []
    for i in range(0,3):
        c.append(a[i] - b[i])
    return(c)

#Finds the normal vector from the sphere collision
def findN(ray,t,sp):
    line = []
    point = [0,0,0,1]
    
    for i in range(0,3):
        line.append(ray.inverseL[i]*t+ray.inverseP[i])


    line.append(0)

    N = ImageClasses.normal(point,line)
    N.inverseTranspose(sp.InverseT,sp.InverseS)
    
    newPoint = [sp.pos[0],sp.pos[1],sp.pos[2]]
    newLine = [N.line[0]*t+N.point[0]-newPoint[0],N.line[1]*t+N.point[1]-newPoint[1],N.line[2]*t+N.point[2]-newPoint[2]]


    return ImageClasses.shadowRay(newPoint,newLine)

#Returns the sphere and intersection t based on the smallest t value
def minI(list):
    x = list[0]

    if len(list) > 1:
        for el in list:
            if(x[0] > el[0]):
               x = el
        return x
    return list[0]


#Makes sure that no colour is greater than 1
def capAtOne(list):
    for i in range(0,len(list)):
        if list[i] > 1:
            list[i] = 1
    return list

#tests if a ray intersects with a sphere
def testIntersecton(r,sp):
    S = r.inverseP
    c = r.inverseL
    r = 1
    A = np.dot(c,c)
    B = np.dot(S,c)
    C = np.dot(S,S) - r**2
    D = (B*B - A*C)
    if(C ==0):
        return False

    
    if(D == 0):
        return False
    elif(D > 0):
            quadp = -B/A + math.sqrt(D)/abs(C*C)
            quadn = -B/A - math.sqrt(D)/abs(C*C)
            x = quadp
            y = quadn
            if(x < y):
                return x
            else:
                return y
    elif(D < 0):
        return False
    return False

#The "Main" ray tracing function, max depth is 3 starting at 0 (4 passes of the function)
def TraceRays(data,spheres,ray,depth):

    #recursive stop case    
    if depth == 3:
        return [0,0,0]

    #Tests intersection for each sphere in the scene
    intersections = []
    for sphere in spheres:
        ray.inverseTransform(sphere)
        inter = testIntersecton(ray,sphere)
        
        if inter != False:
            intersections.append([inter,sphere])
            
    #If there is an intersection(s) get colour from sphere else return background colour
    if len(intersections) != 0:

        #getting the sphere that was intersected
        t,cSphere = minI(intersections)

        if depth == 0:
            #test if intersecting with plane
            if t <1 and t > 0:
                return [cSphere.colour[0]*data.backgroundA[0]*cSphere.ks[0],cSphere.colour[1]*data.backgroundA[1]*cSphere.ks[0],cSphere.colour[2]*data.backgroundA[2]*cSphere.ks[0]]
        
            #test if behind the plane        
            if t < 0:
                return data.backgroundA
            
        #setting up parameters for lighting ect 
        newstart = ray.line * t + ray.point
        sphereC = [0,0,0]
        sphereC = cSphere.colour
        finalC = [0,0,0]
        ray.inverseTransform(cSphere)
        n = findN(ray,t,cSphere)
        N = [n.line[0],n.line[1],n.line[2]]
        N = [N[0]/vecLength(N),N[1]/vecLength(N),N[2]/vecLength(N)]
        N = np.array(N)
        
        V = [ray.line[0]/vecLength(ray.line),ray.line[1]/vecLength(ray.line),ray.line[2]/vecLength(ray.line)]
        V = np.array(V)

        Clocal = [0,0,0]

        

        for light in data.lights:
            #building Shadow Rays
            lightpos = [light[1][0],light[1][1],light[1][2]]
            sr = ImageClasses.shadowRay(newstart,lightpos-newstart[0:3])
            
            blocked = []
            #testing if a SR is blocked
            for sphere in spheres:
                sr.inverseTransform(sphere)
                if(sphere.id != cSphere.id):
                    temp = testIntersecton(sr,sphere) 
                    if(temp == False):
                        blocked.append(False)
                    #If distance from the intersection is greater than the distance from the origin dont count as intersection    
                    elif(vecLength(lightpos - (temp*sr.line[0:3] + sr.point[0:3])) > vecLength(sr.line[0:3])):
                        blocked.append(False)

            #If not blocked calculate diffuse and specular
            if(len(blocked) == len(spheres)-1):

                L = [sr.line[0]/vecLength(sr.line),sr.line[1]/vecLength(sr.line),sr.line[2]/vecLength(sr.line)]
                L = np.array(L)
                
                R = 2*(N.dot(L))*N-L
                R = np.array(R)


                Clocal[0] = Clocal[0] + (light[2][0]*(N.dot(L))*sphereC[0]*cSphere.ks[1]) + cSphere.ks[2]*light[2][0]*(R.dot(V))**cSphere.n
                Clocal[1] = Clocal[1] + (light[2][1]*(N.dot(L))*sphereC[1]*cSphere.ks[1]) + cSphere.ks[2]*light[2][1]*(R.dot(V))**cSphere.n
                Clocal[2] = Clocal[2] + (light[2][2]*(N.dot(L))*sphereC[2]*cSphere.ks[1]) + cSphere.ks[2]*light[2][2]*(R.dot(V))**cSphere.n
        
        #removes the current intersected sphere from the list so the reflected ray cant intersect it
        newSphereset = []
        for sphere in spheres:
            if sphere.id != cSphere.id:
                newSphereset.append(sphere)
        #Setting up the reflected ray
        v = -2*N.dot(ray.line[0:3])*N + V
        reflected = ImageClasses.shadowRay(newstart,v)
        ref = TraceRays(data,newSphereset,reflected,depth+1)
        #Final colour calculations
        finalC[0] = sphereC[0]*data.backgroundA[0]*intersections[0][1].ks[0] + Clocal[0] + cSphere.ks[3]*ref[0]
        finalC[1] = sphereC[1]*data.backgroundA[1]*intersections[0][1].ks[0] + Clocal[1] + cSphere.ks[3]*ref[1]
        finalC[2] = sphereC[2]*data.backgroundA[2]*intersections[0][1].ks[0] + Clocal[2] + cSphere.ks[3]*ref[2]

        capAtOne(finalC)
        return finalC
    #Base case is theres no intersection found
    if depth > 0:
        return [0,0,0]
    else:
        return data.backgroundC 

#############################Main#############################
def main():

    #parsing the file
    filename = sys.argv[1]
    file = open(filename,'r')
    params = (file.read())
    params = params.split('\n')
    terms = []
    for element in params:
        terms.append(element.split())
    file.close()

    #parsing the data within the file
    terms = [el for el in terms if el != []]
    plane = []
    res = []
    spheres = []
    lights = []
    back = []
    amb = []
    outfilename = ''
    for el in terms:
        if(el[0] == "NEAR" or el[0] == "LEFT" or el[0] == "RIGHT" or el[0] == "BOTTOM" or el[0] == "TOP"):
            plane.append([el[0],float(el[1])])
        
        if(el[0] == "RES"):
            res = [int(el[1]),int(el[2])]

        if(el[0] == "SPHERE"):
            pos = [float(el[2]),float(el[3]),float(el[4])]
            scale = [float(el[5]),float(el[6]),float(el[7])]
            color = [float(el[8]),float(el[9]),float(el[10])]
            ks = [float(el[11]),float(el[12]),float(el[13]),float(el[14])]
            n = float(el[15])
            name = el[1]
            spheres.append(ImageClasses.sphere(pos,scale,color,ks,n,name))
            
        if(el[0] == "LIGHT"):
            l = [el[1],[float(el[2]),float(el[3]),float(el[4])],[float(el[5]),float(el[6]),float(el[7])]]
            lights.append(l)

        if(el[0] == "BACK"):
            back = [float(el[1]),float(el[2]),float(el[3])]

        if(el[0] == "AMBIENT"):
            amb = [float(el[1]),float(el[2]),float(el[3])]
        
        if(el[0] == "OUTPUT"):
            outfilename = el[1]

    scene = ImageClasses.image(plane,res,lights,back,amb)

    #outputting stuff
    outfile = "P3\n#testing\n" + str(scene.res[0]) + " " + str(scene.res[1]) + "\n255 \n"  
    
    for sphere in spheres:
        sphere.undoTransforms()

    #Running the ray tracer for each pixel in the scene
    for j in range(int(scene.res[0]),0,-1):
        tempstring = ""
        for i in range(0,int(scene.res[1])):
            temp = TraceRays(scene,spheres,ImageClasses.startRay(j,i,scene),0)
            tempstring = tempstring + str(int(temp[0]*255)) + " "
            tempstring = tempstring + str(int(temp[1]*255)) + " "
            tempstring = tempstring + str(int(temp[2]*255)) + " "
        tempstring = tempstring + "\n"
        outfile = outfile + tempstring

    fileout = open(outfilename, 'w')
    fileout.write(outfile)
    fileout.close()

if(__name__ == "__main__"):
    main()
#Lab2 graficas
#Jessica Ortiz 20192

import struct
from obj import Obj
from collections import namedtuple
import math
import random


V2 = namedtuple('Vertex2', ['x', 'y'])

V3 = namedtuple('Vertex3', ['x', 'y', 'z'])


def char(c):
    # 1 byte
    return struct.pack('=c', c.encode('ascii'))

def word(w):
    # 2 bytes
    return struct.pack('=h', w)

def dword(w):
    # 4 bytes
    return struct.pack('=l', w) 



def barycentric(A, B, C, P):
    bary = cross(
    V3(C.x - A.x, B.x - A.x, A.x - P.x), 
    V3(C.y - A.y, B.y - A.y, A.y - P.y)
    )

    if abs(bary[2]) < 1:
        return -1, -1, -1 

    return (
    1 - (bary[0] + bary[1]) / bary[2], 
    bary[1] / bary[2], 
    bary[0] / bary[2]
    )
    
#suma
def sum(v0, v1):
    return V3(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z)

#resta
def sub(v0, v1):
    return V3(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z)

#largo
def length(v0):
    return (v0.x**2 + v0.y**2 + v0.z**2)**0.5

#normal
def norm(v0):
    v0length = length(v0)

    if not v0length:
        return V3(0, 0, 0)

    return V3(v0.x/v0length, v0.y/v0length, v0.z/v0length)

#multplicacion
def mul(v0, k):
    return V3(v0.x * k, v0.y * k, v0.z *k)

#produco punto
def dot(v0, v1):
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z

#producto cruz
def cross(v0, v1):
    return V3(
    v0.y * v1.z - v0.z * v1.y,
    v0.z * v1.x - v0.x * v1.z,
    v0.x * v1.y - v0.y * v1.x,
    )

#bounding box
def bbox(*vertices):
    xs = [ vertex.x for vertex in vertices ]
    ys = [ vertex.y for vertex in vertices ]
    xs.sort()
    ys.sort()

    return V2(xs[0], ys[0]), V2(xs[-1], ys[-1])


class Render(object):
    def color(self, r, g, b):
        return bytes([int(b*self.color_range), int(g*self.color_range), int(r*self.color_range)])

    def glInit(self):
        self.color_range = 255
        self.current_color_clear = self.color(0,0,0)
        self.current_color = self.color(1, 1, 1)

    def glCreateWindow(self, width, height):
        self.width = width
        self.height = height
        self.glClear()

    def glViewPort(self, x, y, width, height):
        if x >= 0 and y >= 0 and width >= 0 and height >= 0 and x + width <= self.width and y + height <= self.height:
            self.xvp = x
            self.yvp = y
            self.wvp = width
            self.hvp = height

    def glClear(self):
        self.framebuffer = [
            [self.current_color_clear for x in range(self.width)]
            for y in range(self.height)
        ]
        self.zbuffer = [
            [-float('inf') for x in range(self.width)]
            for y in range(self.height)
        ]

    def glClearColor(self, r, g, b):
        self.current_color_clear = self.color(r, g, b)

    def glColor(self, r, g, b):
        self.current_color = self.color(r, g, b)

    def glPoint(self, x, y, color = None):
        self.framebuffer[y+self.yvp][x+self.xvp] = color or self.current_color

    def glVertex(self, x, y, color = None):
        if x >= -1 and x <= 1 and y >= -1 and y <= 1:
            self.framebuffer[int(self.yvp + y * (self.hvp / 2) + self.hvp / 2)][int(self.xvp + x * (self.wvp / 2) + self.wvp / 2)] = color or self.current_color
            
    def glLine(self, x0, y0, x1, y1):
        x0 = round(x0*self.wvp)
        y0 = round(y0*self.hvp)
        x1 = round(x1*self.wvp)
        y1 = round(y1*self.hvp)
        dy = abs(y1 - y0)
        dx = abs(x1 - x0)
        steep = dy > dx
        if steep:
            x0, y0 = y0, x0
            x1, y1 = y1, x1

            dy = abs(y1 - y0)
            dx = abs(x1 - x0)

        offset = 0 * 2 * dx
        threshold = 0.5 * 2 * dx
        y = y0
        points = []
        for x in range(x0, x1):
            if steep:
                points.append([y/self.wvp, x/self.hvp])
            else:
                points.append([x/self.wvp, y/self.hvp])

            offset += (dy/dx) * 2 * dx
            if offset >= threshold:
                y += 1 if y0 < y1 else -1
                threshold += 1 * 2 * dx
        for point in points:
            self.glVertex(*point)
    def line(self, x0, y0, x1, y1):
        dy = abs(y1 - y0)
        dx = abs(x1 - x0)
        steep = dy > dx
        if steep:
            x0, y0 = y0, x0
            x1, y1 = y1, x1

            dy = abs(y1 - y0)
            dx = abs(x1 - x0)

        offset = 0 * 2 * dx
        threshold = 0.5 * 2 * dx
        y = y0
        points = []
        x = x0
        cont = 1
        if x0 > x1:
            cont = -1
        while x != x1:
            if steep:
                points.append([y, x])
            else:
                points.append([x, y])

            offset += (dy/dx) * 2 * dx
            if offset >= threshold:
                y += 1 if y0 < y1 else -1
                threshold += 1 * 2 * dx
            x = x + cont
        for point in points:
            self.glPoint(*point)


    def triangle(self, A, B, C,color=None):
        bbox_min, bbox_max = bbox(A, B, C)

        for x in range(bbox_min.x, bbox_max.x + 1):
            for y in range(bbox_min.y, bbox_max.y + 1):
                w, v, u = barycentric(A, B, C, V2(x, y))
                if w < 0 or v < 0 or u < 0:
                    continue
                color = self.shader(x, y, self.wvp, self.hvp)
                z = A.z * w + B.z * v + C.z * u
                if z > self.zbuffer[y][x]:
                    self.glPoint(x, y, color)
                    self.zbuffer[y][x] = z

    def glObjModel(self, filename, translate=(0, 0, 0), scale=(1, 1, 1)):
        model = Obj(filename)

        light = V3(0,0,1)
        
        for face in model.faces:
            vcount = len(face)

            if vcount == 3:
                f1 = face[0][0] - 1
                f2 = face[1][0] - 1
                f3 = face[2][0] - 1

                a = self.transform(model.vertices[f1], translate, scale)
                b = self.transform(model.vertices[f2], translate, scale)
                c = self.transform(model.vertices[f3], translate, scale)

                normal = norm(cross(sub(b, a), sub(c, a)))
                intensity = dot(normal, light)
                grey = intensity
                if grey < 0:
                    continue  
                
                self.triangle(a, b, c, self.color(grey, grey, grey))
            else:
                # assuming 4
                f1 = face[0][0] - 1
                f2 = face[1][0] - 1
                f3 = face[2][0] - 1
                f4 = face[3][0] - 1   

                vertices = [
                    self.transform(model.vertices[f1], translate, scale),
                    self.transform(model.vertices[f2], translate, scale),
                    self.transform(model.vertices[f3], translate, scale),
                    self.transform(model.vertices[f4], translate, scale)
                ]

                normal = norm(cross(sub(vertices[0], vertices[1]), sub(vertices[1], vertices[2])))
                intensity = dot(normal, light)
                grey = intensity
                if grey < 0:
                    #no se pinta esta cara
                    continue 
                #vertices ordenados
                A, B, C, D = vertices 
                
                self.triangle(A, B, C, self.color(grey, grey, grey))
                self.triangle(A, C, D, self.color(grey, grey, grey))

    def fillPolygon(self, texto, traslado):
        puntos = texto[:-1].split(') ')
        separado = [punto[1:].split(', ') for punto in puntos]
        lista = []
        for punto in separado:
            lista.append([str(int(punto[0])+traslado[0]),str(int(punto[1])+traslado[1])])
        cont = 0
        minx = 1000000
        miny = 1000000
        maxx = 0
        maxy = 0
        while cont < len(lista):
            self.line(int(lista[cont][0]), int(lista[cont][1]), int(lista[(cont+1) % len(lista)][0]), int(lista[(cont+1) % len(lista)][1]))
            if minx>int(lista[cont][0]):
                minx = int(lista[cont][0])
            if maxx<int(lista[cont][0]):
                maxx = int(lista[cont][0])
            if miny>int(lista[cont][1]):
                miny = int(lista[cont][1])
            if maxy<int(lista[cont][1]):
                maxy = int(lista[cont][1])
            cont = cont + 1
        bandera = False
        for x in range(minx,maxx+1):
            for y in range(miny,maxy+1):
                if self.framebuffer[y][x] == self.current_color and not ([str(x),str(y)] in lista) and self.framebuffer[y+1][x] != self.current_color :
                    valor = False
                    for i in range(y+1, maxy+1):
                        if self.framebuffer[i][x] == self.current_color:
                            valor = True 
                    bandera = valor
                if bandera:
                    self.framebuffer[y][x] = self.current_color


    # Función para crear la imagen
    def glFinish(self, filename):
        f = open(filename, 'bw')
        # Header
        f.write(char('B'))
        f.write(char('M'))
        f.write(dword(14+40+3*(self.width*self.height)))
        f.write(dword(0))
        f.write(dword(14+40))

        #image header
        f.write(dword(40))
        f.write(dword(self.width))
        f.write(dword(self.height))
        f.write(word(1))
        f.write(word(24))
        f.write(dword(0))
        f.write(dword(self.width*self.height*3))
        f.write(dword(0))
        f.write(dword(0))
        f.write(dword(0))
        f.write(dword(0))

        #pixel data
        for y in range(self.height):
            for x in range(self.width):
                f.write(self.framebuffer[y][x])

        f.close()

    def transform(self, vertex, translate=(0, 0, 0), scale=(1, 1, 1)):
    # returns a vertex 3, translated and transformed
        return V3(
            round((vertex[0] + translate[0]) * scale[0]),
            round((vertex[1] + translate[1]) * scale[1]),
            round((vertex[2] + translate[2]) * scale[2])
        )

    def shader(self, x, y, maxx, maxy):
        color = [23, 54, 158]
        point = [670, 470]
        #norte america

        # y < 1080 tamaño de la pantalla menos donde se quiere que empiece la linea que se pinta, el otro y>1080 - donde va a terminar la linea definiedo el grosor
        # y el de x donde se quiere que empieze la linea de izquierda a derecha
        #y se le resta un numero random para que las lineas no queden tan marcadas, sino que se desvanezcan un poco
        if y < 1080 and y > 1080 - 170 - random.randint(0,5) and x < 1000- random.randint(0,10):
            color = [125, 206, 160]
            #nube
        if y < 1080-170 and y > 1080 - 195 - random.randint(0,5) and x < 1100- random.randint(0,10) and x >1020- random.randint(0,10):
            color = [255,255,255]

        if y < 1080 - 100 and y > 1080 - 185 - random.randint(0,5) and x < 980- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 180 and y > 1080 - 235 - random.randint(0,5) and x < 920- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 230 and y > 1080 - 285 - random.randint(0,5) and x < 900- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 280 and y > 1080 - 335 - random.randint(0,5) and x < 860 - random.randint(0,10) and x > 660- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 330 and y > 1080 - 370 - random.randint(0,5) and x < 840 - random.randint(0,10) and x > 690- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 365 and y > 1080 - 385 - random.randint(0,5) and x < 820 - random.randint(0,10) and x > 695- random.randint(0,10):
            color = [125, 206, 160]
            #nube
        if y < 1080 - 365 and y > 1080 - 385 - random.randint(0,5) and x < 2000 - random.randint(0,10) and x > 1060- random.randint(0,10):
            color = [255,255,255]
        if y < 1080 - 380 and y > 1080 - 410 - random.randint(0,5) and x < 2000 - random.randint(0,10) and x > 1080- random.randint(0,10):
            color = [255,255,255]

        if y < 1080 - 380 and y > 1080 - 405 - random.randint(0,5) and x < 800 - random.randint(0,10) and x > 710- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 400 and y > 1080 - 420 - random.randint(0,5) and x < 800 - random.randint(0,10) and x > 730- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 415 and y > 1080 - 440 - random.randint(0,5) and x < 800 - random.randint(0,10) and x > 730- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 430 and y > 1080 - 460 - random.randint(0,5) and x < 820 - random.randint(0,10) and x > 770- random.randint(0,10):
            color = [125, 206, 160]
        #centro america
        if y < 1080 - 455 and y > 1080 - 480 - random.randint(0,5) and x < 840 - random.randint(0,10) and x > 770- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 475 and y > 1080 - 500 - random.randint(0,5) and x < 860 - random.randint(0,10) and x > 770- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 450 and y > 1080 - 470 - random.randint(0,5) and x < 970 - random.randint(0,10) and x > 880- random.randint(0,10):
            color = [125, 206, 160]
            #nube
        if y < 1080 - 480 and y > 1080 - 505 - random.randint(0,5) and x < 1200 - random.randint(0,10) and x > 1020- random.randint(0,10):
            color = [255,255,255]

        if y < 1080 - 495 and y > 1080 - 515 - random.randint(0,5) and x < 880 - random.randint(0,10) and x > 790- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 510 and y > 1080 - 530 - random.randint(0,5) and x < 880 - random.randint(0,10) and x > 810- random.randint(0,10):
            color = [125, 206, 160]
        
        #sur america
        if y < 1080 - 525 and y > 1080 - 550 - random.randint(0,5) and x < 1100 - random.randint(0,10) and x > 880- random.randint(0,10):
            color = [125, 206, 160] 
        if y < 1080 - 545 and y > 1080 - 570 - random.randint(0,5) and x < 1130 - random.randint(0,10) and x > 880- random.randint(0,10):
            color = [125, 206, 160] 
        if y < 1080 - 565 and y > 1080 - 595 - random.randint(0,5) and x < 1145 - random.randint(0,10) and x > 860- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 590 and y > 1080 - 620 - random.randint(0,5) and x < 1185 - random.randint(0,10) and x > 850- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 615 and y > 1080 - 650 - random.randint(0,5) and x < 1200 - random.randint(0,10) and x > 850- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 635 and y > 1080 - 680 - random.randint(0,5) and x < 1220 - random.randint(0,10) and x > 860- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 675 and y > 1080 - 710 - random.randint(0,5) and x < 1190 - random.randint(0,10) and x > 880- random.randint(0,10):
            color = [125, 206, 160] 
        if y < 1080 - 705 and y > 1080 - 740 - random.randint(0,5) and x < 1150 - random.randint(0,10) and x > 920- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 735 and y > 1080 - 780 - random.randint(0,5) and x < 1110 - random.randint(0,10) and x > 960- random.randint(0,10):
            color = [125, 206, 160]
        if y < 1080 - 775 and y > 1080 - 810 - random.randint(0,5) and x < 1080 - random.randint(0,10) and x > 990- random.randint(0,10):
            color = [125, 206, 160]
            #nube
        if y < 1080 - 775 and y > 1080 - 810 - random.randint(0,5) and x < 1150 - random.randint(0,10) and x > 1000- random.randint(0,10):
            color = [255,255,255]

        if y < 1080 - 805 and y > 1080 - 840 - random.randint(0,5) and x < 1050 - random.randint(0,10) and x > 990- random.randint(0,10):
            color = [125, 206, 160] 
        if y < 1080 - 835 and y > 1080 - 870 - random.randint(0,5) and x < 1010 - random.randint(0,10) and x > 990- random.randint(0,10):
            color = [125, 206, 160]
            #nube
        if y < 1080 - 835 and y > 1080 - 870 - random.randint(0,5) and x < 1100 - random.randint(0,10) and x > 1030- random.randint(0,10):
            color = [255,255,255] 
    
        #polo
        if y < 1080 - 910 + random.randint(0,10) and y > 1080 - 1000 - random.randint(0,10): 
            color = [223, 228, 247]


        d = math.dist([point[0], point[1]], [x, y]) / 720
        d = 1-d
        color = [c*d for c in color]
        return self.glColor(color[0]/self.color_range,color[1]/self.color_range,color[2]/self.color_range)
    

#Lab2 graficas
#Jessica Ortiz 20192

import struct
from obj import Obj
from collections import namedtuple



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


    def glFinish(self, filename):
        f = open(filename, 'bw')
        f.write(char('B'))
        f.write(char('M'))
        f.write(dword(14+40+3*(self.width*self.height)))
        f.write(dword(0))
        f.write(dword(14+40))

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

        for y in range(self.height):
            for x in range(self.width):
                f.write(self.framebuffer[y][x])

        f.close()

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
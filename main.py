from gl import Render

r = Render()
r.glInit()
r.glCreateWindow(1920, 1080)
r.glViewPort(0, 0, 1920, 1080)
r.glObjModel('./sphere.obj',[1.2,0.65,10],[800,800,1])

r.glFinish('tierra.bmp')
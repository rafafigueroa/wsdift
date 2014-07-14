import pyglet
from pyglet.gl import *
import time

window = pyglet.window.Window()

vertex_list = pyglet.graphics.vertex_list(3,
        ('v2i', (200, 100,
                 150, 200,
                 200, 150)),
        ('c3B',(200, 100,0,
                 150, 200,0,
                 200, 150,0))
            )

@window.event
def on_draw():
    vertex_list.draw(pyglet.gl.GL_TRIANGLES)


pyglet.app.run()
vertex_list.vertices = [200, 250, 150, 145,234,123]
vertex_list.draw(pyglet.gl.GL_TRIANGLES)
time.sleep(1)
vertex_list.vertices = [20, 150, 150, 14,234,123]
vertex_list.draw(pyglet.gl.GL_TRIANGLES)
time.sleep(1)
vertex_list.vertices = [122, 15, 15, 14,0,123]
vertex_list.draw(pyglet.gl.GL_TRIANGLES)

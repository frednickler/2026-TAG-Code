#!/usr/bin/env python3
"""Standalone process that renders a 3-D cube using PyOpenGL/freeglut.

The main visualiser (imu_visualizer.py) spawns this process and passes a
multiprocessing.Queue that streams the latest orientation quaternion
[w, x, y, z].  The queue is polled inside a GLUT timer so that the cube
reflects the most recent orientation without blocking the event loop.

Running GLUT in a dedicated process ensures it owns the process's main
thread – a hard requirement on macOS/X11.  This avoids the crashes seen
when GLUT is started from a worker thread in the main visualiser.
"""
from __future__ import annotations

import sys
import math
from multiprocessing import Queue
from typing import List

from OpenGL.GL import *   # noqa: F401, F403  (PyOpenGL style)
from OpenGL.GLU import *  # noqa: F401, F403
from OpenGL.GLUT import *  # noqa: F401, F403

# -------------- Quaternion → rotation-matrix helpers -------------- #

def quaternion_to_rotation_matrix(q: List[float]):
    """Convert quaternion [w, x, y, z] to a 4×4 column-major matrix."""
    w, x, y, z = q
    return [
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),     0,
        2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),     0,
        2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y), 0,
        0,                       0,                       0,                       1,
    ]

# ----------------------- OpenGL drawing -------------------------- #

# Shared state updated from queue
active_quaternion: List[float] = [1.0, 0.0, 0.0, 0.0]


def draw_cube():
    """Draw a coloured cube using the current quaternion."""
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    gluLookAt(0.0, 0.0, 5.0,   # Eye
              0.0, 0.0, 0.0,   # Centre
              0.0, 1.0, 0.0)   # Up

    # Apply IMU orientation
    glMultMatrixf(quaternion_to_rotation_matrix(active_quaternion))

    glBegin(GL_QUADS)
    # Front (red)
    glColor3f(1, 0, 0); glVertex3f(-1, -1, 1); glVertex3f(1, -1, 1); glVertex3f(1, 1, 1); glVertex3f(-1, 1, 1)
    # Back (green)
    glColor3f(0, 1, 0); glVertex3f(-1, -1, -1); glVertex3f(-1, 1, -1); glVertex3f(1, 1, -1); glVertex3f(1, -1, -1)
    # Top (blue)
    glColor3f(0, 0, 1); glVertex3f(-1, 1, -1); glVertex3f(-1, 1, 1); glVertex3f(1, 1, 1); glVertex3f(1, 1, -1)
    # Bottom (yellow)
    glColor3f(1, 1, 0); glVertex3f(-1, -1, -1); glVertex3f(1, -1, -1); glVertex3f(1, -1, 1); glVertex3f(-1, -1, 1)
    # Right (cyan)
    glColor3f(0, 1, 1); glVertex3f(1, -1, -1); glVertex3f(1, 1, -1); glVertex3f(1, 1, 1); glVertex3f(1, -1, 1)
    # Left (magenta)
    glColor3f(1, 0, 1); glVertex3f(-1, -1, -1); glVertex3f(-1, -1, 1); glVertex3f(-1, 1, 1); glVertex3f(-1, 1, -1)
    glEnd()

    glutSwapBuffers()

# ----------------------- GLUT setup ------------------------------ #

def _timer_cb(value):
    """Post redisplay and reschedule timer."""
    glutPostRedisplay()
    glutTimerFunc(16, _timer_cb, 0)  # ~60 Hz

# ---------------------- Main entrypoint -------------------------- #

def run(quat_queue: Queue):
    """Entry point called by the parent process."""
    global active_quaternion

    # GLUT initialisation – must be in process main thread
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(500, 500)
    glutCreateWindow(b"IMU 3D Orientation")

    glClearColor(0, 0, 0, 0)
    glEnable(GL_DEPTH_TEST)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1, 0.1, 100)
    glMatrixMode(GL_MODELVIEW)

    glutDisplayFunc(draw_cube)
    glutTimerFunc(16, _timer_cb, 0)

    # Queue-polling loop integrated via GLUT idle callback
    def _idle():
        global active_quaternion
        try:
            while True:  # drain queue to keep the latest quaternion
                active_quaternion = quat_queue.get_nowait()
        except Exception:
            pass  # empty
        return 0

    glutIdleFunc(_idle)

    print("GLUT cube window running in PID", os.getpid())
    glutMainLoop()

# Entry when launched directly (mainly for debugging)
if __name__ == "__main__":
    from multiprocessing import Queue

    q = Queue(maxsize=1)
    p = Process(target=run, args=(q,))
    p.start()

    # feed dummy rotations
    import time
    angle = 0
    while True:
        qw = math.cos(angle / 2)
        qx = 0
        qy = math.sin(angle / 2)
        qz = 0
        try:
            q.put_nowait([qw, qx, qy, qz])
        except Exception:
            pass
        angle += 0.01
        time.sleep(0.02)

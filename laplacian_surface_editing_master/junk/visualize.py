#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import networkx as nx
import trimesh
from utils import *
import scipy.sparse


# In[2]:


mesh = trimesh.load('./meshes/bunny.ply', process=False)


# In[3]:


mesh_edited = trimesh.load('./exports/bunny_20191220154636.ply', process=False)


# In[13]:


scene = trimesh.Scene([
    trimesh.points.PointCloud(mesh.vertices, colors=[(200, 200, 200, 100) for i in range(len(mesh.vertices))]),
    mesh_edited
])
print(type(scene))

with open('render_orig.png', 'wb') as f:
    f.write(scene.save_image(visible=True))


# In[ ]:





#!/usr/bin/env python3
"""Genera las franjas arcoiris de Rainbow Road (meshes/pistas/rainbow/rainbow_0..6.stl).
Uso: python3 scripts/generar_rainbow.py meshes/pistas/rainbow_road.stl meshes/pistas/rainbow
Requiere: pip install trimesh shapely scipy mapbox-earcut"""
import sys, numpy as np, trimesh, shapely
from shapely.geometry import Polygon, box
from shapely.ops import unary_union
src, outdir = sys.argv[1:3]
m = trimesh.load(src)
up = m.face_normals[:,2] > 0.9
cz = m.triangles_center[:,2]
road = unary_union([Polygon(t[:,:2]) for t in m.triangles[up & (abs(cz-0.05)<0.01)]]).buffer(0)
print('road area %.2f m2' % road.area)
C = 0.02                                   # grid cell [m]
x0,y0,x1,y1 = road.bounds
xs = np.arange(x0, x1, C) + C/2; ys = np.arange(y0, y1, C) + C/2
X, Y = np.meshgrid(xs, ys)
inside = shapely.contains_xy(road, X, Y)
cost = np.where(inside, 1.0, -1.0)         # negative = impassable
# start line on the top straight at x=-2.0 (spawn), wall just behind it so distance runs one way (+x) around the loop
lane = (Y > 3.9) & (Y < 4.7)
cost[lane & (X >= -2.06) & (X < -2.0)] = -1
starts = list(zip(*np.where(lane & inside & (X >= -2.0) & (X < -1.98))))
# Harmonic coordinate: Laplace(u)=0 on the road, u=0 just after the start line, u=1 just before it,
# zero-flux on the walls -> its level lines cross the lane perpendicular to the walls.
import scipy.sparse as sp, scipy.sparse.linalg as spl
act = inside & (cost > 0)
start = act & lane & (X >= -2.0) & (X < -1.98)
end   = act & lane & (X >= -2.08) & (X < -2.06)
ids = -np.ones(act.shape, int); ids[act] = np.arange(act.sum())
R, Cc, V = [], [], []; b = np.zeros(act.sum())
for (i, j) in zip(*np.where(act)):
    r = ids[i, j]
    if start[i, j] or end[i, j]:
        R.append(r); Cc.append(r); V.append(1.0); b[r] = 1.0 if end[i, j] else 0.0; continue
    n = 0
    for di, dj in ((1,0),(-1,0),(0,1),(0,-1)):
        a, c = i+di, j+dj
        if 0 <= a < act.shape[0] and 0 <= c < act.shape[1] and act[a, c]:
            R.append(r); Cc.append(ids[a, c]); V.append(-1.0); n += 1
    R.append(r); Cc.append(r); V.append(float(n))
A = sp.csr_matrix((V, (R, Cc)), shape=(act.sum(),)*2)
u = np.full(act.shape, np.nan); u[act] = spl.spsolve(A, b)
# remap u -> arc length: on each u-level, ds/du ~ mean(1/|grad u|) across the lane
gy, gx = np.gradient(u, C)
g = np.hypot(gx, gy)
nb = 3000
bins = np.clip((u[act] * nb).astype(int), 0, nb - 1)
inv = 1.0 / np.maximum(g[act], 1e-9)
good = np.isfinite(inv) & (g[act] > 0)
dsdu = np.array([np.median(inv[good & (bins == k)]) if np.any(good & (bins == k)) else np.nan for k in range(nb)])
dsdu = np.interp(np.arange(nb), np.where(np.isfinite(dsdu))[0], dsdu[np.isfinite(dsdu)])
s_edges = np.concatenate([[0], np.cumsum(dsdu / nb)])
L = s_edges[-1]
dist = np.full(act.shape, np.nan)
dist[act] = np.minimum(np.interp(u[act], np.linspace(0, 1, nb + 1), s_edges), L - 1e-6)
print('centreline loop length %.2f m' % L)
ok = act
print('u range', np.nanmin(u), np.nanmax(u))
N = 7
BAND = L / (round(L / 0.25 / N) * N)   # whole number of 7-colour cycles so the seam matches
idx = np.where(ok, np.floor(np.nan_to_num(dist) / BAND), 0).astype(np.int64) % N
# cells behind the start line belong to the end of the loop: give them the last colour
idx[inside & ~ok] = N - 1
for k in range(N):
    sel = inside & (idx == k)
    cells = [box(x-C/2, y-C/2, x+C/2, y+C/2) for x, y in zip(X[sel], Y[sel])]
    poly = unary_union(cells).buffer(0.001).buffer(-0.001).intersection(road).simplify(0.003)
    polys = [poly] if poly.geom_type == 'Polygon' else [g for g in getattr(poly,'geoms',[]) if g.geom_type=='Polygon']
    parts = [trimesh.creation.extrude_polygon(p, 0.001) for p in polys if p.area > 1e-5]
    mesh = trimesh.util.concatenate(parts); mesh.apply_translation([0,0,0.0502])
    mesh.export(f'{outdir}/rainbow_{k}.stl')
    print(k, 'area %.2f m2, %d faces' % (poly.area, len(mesh.faces)))

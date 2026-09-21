"""GPU contact diagnostics matching RobotEngine's generated-contact scope."""
import warp as wp

@wp.kernel
def contacts(nacon:wp.array(dtype=int),world:wp.array(dtype=int),geom:wp.array(dtype=wp.vec2i),dist:wp.array(dtype=float),hook:wp.array(dtype=int),hit:wp.array(dtype=int),minimum:wp.array(dtype=float)):
 i=wp.tid()
 if i<nacon[0]:
  g=geom[i];w=world[i]
  if g[0]>=0 and g[1]>=0 and w>=0:
   if hook[g[0]]!=0 or hook[g[1]]!=0:
    wp.atomic_max(hit,w,1)
    wp.atomic_min(minimum,w,dist[i])

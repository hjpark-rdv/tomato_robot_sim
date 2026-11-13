"""Keep solid CAD covered without filling the empty workspace above the deck."""
import unittest

import numpy as np
import trimesh

from preview_side_collision import component_bounds, sliced_bounds


def contains(bounds, point):
    return any(np.all(point >= a-1e-9) and np.all(point <= b+1e-9) for a,b in bounds)


class SideCollisionTests(unittest.TestCase):
    def test_disconnected_console_and_deck_do_not_fill_the_gap(self):
        deck=trimesh.creation.box(extents=[.6,1.4,.1])
        console=trimesh.creation.box(extents=[.5,.2,.7])
        console.apply_translation([0.,.6,.4])
        mesh=trimesh.util.concatenate([deck,console])
        bounds=component_bounds(mesh)
        self.assertEqual(len(bounds),2)
        self.assertTrue(all(contains(bounds,p) for p in mesh.vertices))
        self.assertFalse(contains(bounds,np.array([0.,0.,.4])))
        self.assertTrue(contains(bounds,np.array([0.,.6,.4])))

    def test_sliced_foot_covers_crossing_triangles_but_not_empty_corner(self):
        foot=trimesh.creation.box(extents=[.02,1.,.1]);foot.apply_translation([0.,.5,.05])
        back=trimesh.creation.box(extents=[.02,.1,.9]);back.apply_translation([0.,.95,.55])
        mesh=trimesh.util.concatenate([foot,back]);bounds=sliced_bounds(mesh,height=.025)
        # Long vertical faces cross many bands containing no source vertices.
        samples=np.concatenate([mesh.vertices,mesh.triangles_center,
            *[(1-u)*mesh.vertices[mesh.edges_unique[:,0]]+u*mesh.vertices[mesh.edges_unique[:,1]]
              for u in np.linspace(0.,1.,37)]])
        self.assertTrue(all(contains(bounds,p) for p in samples))
        self.assertFalse(contains(bounds,np.array([0.,.5,.5])))
        self.assertTrue(contains(bounds,np.array([0.,.95,.5])))


if __name__=='__main__':unittest.main()

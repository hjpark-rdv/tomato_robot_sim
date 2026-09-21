"""Cached evaluation of the original visual skin weights; no physics writes."""
import numpy as np
from scipy.sparse import coo_matrix


class DisplaySkin:
    def __init__(self, points, rest_positions, ids, weights, normals):
        count=len(points);bodies=len(rest_positions)
        vertex,influence=np.nonzero(weights)
        bone=ids[vertex,influence];weight=weights[vertex,influence]
        local=points[vertex]-rest_positions[bone]
        coefficients=np.column_stack((local,np.ones(len(local))))*weight[:,None]
        rows=np.repeat(vertex,4);columns=(bone[:,None]*4+np.arange(4)).ravel()
        self.points=coo_matrix((coefficients.ravel(),(rows,columns)),shape=(count,bodies*4)).tocsr()
        normal_coefficients=normals[vertex]*weight[:,None]
        rows=np.repeat(vertex,3);columns=(bone[:,None]*3+np.arange(3)).ravel()
        self.normals=coo_matrix((normal_coefficients.ravel(),(rows,columns)),shape=(count,bodies*3)).tocsr()

    @staticmethod
    def transforms(rotations, positions):
        linear=rotations.transpose(0,2,1)
        affine=np.concatenate((linear,positions[:,None,:]),axis=1)
        return affine.reshape(-1,3),linear.reshape(-1,3)

    def evaluate(self, transforms):
        affine,linear=transforms
        return self.points@affine,self.normals@linear

import numpy as np 
import copy 

class VectorUtilsNumpyWrapper:

    # Vector Norm/Magnitude/Length
    def vector_length_3D( self, v ):
        return np.linalg.norm( copy.deepcopy(v) )
     
    def unit_vector_3D( self, v ):
        vcopy = copy.deepcopy( v )
        return list ( vcopy / self.vector_length_3D( vcopy ) )
    
    def vector_subtraction_3D( self, v1, v2 ):
        return list ( np.subtract( copy.deepcopy(v1), copy.deepcopy(v2) ) ) 
    
    def dot_product( self, v1, v2 ):
        return np.dot( copy.deepcopy(v1), copy.deepcopy(v2) )
    
    def cross_product_3D( self, v1, v2 ):
        return list ( np.cross(  copy.deepcopy(v1), copy.deepcopy(v2) ) )
    
    def angle_between_two_vectors( self, v1, v2 ):
        v1_unit = self.unit_vector_3D( copy.deepcopy(v1)  )
        v2_unit = self.unit_vector_3D( copy.deepcopy(v2)  )
        dot_product = self.dot_product( v1_unit, v2_unit )
        return np.arccos( np.clip( dot_product, -1.0, 1.0 ) )
    
    def rad2deg( self, rad ):
        return np.degrees( rad )
    
    def deg2rad( self, deg ):
        return np.radians( deg )
        
class VectorUtils:
    
    # Vector Norm/Magnitude/Length
    def vector_length_3D( self, v ):
        vcopy = copy.deepcopy( v )
        return np.sqrt( vcopy[0]*vcopy[0] + vcopy[1]*vcopy[1] + vcopy[2]*vcopy[2] )

    def unit_vector_3D( self, v ):
        vcopy = copy.deepcopy( v )
        vector_length = self.vector_length_3D( vcopy )
        result = [ 0.0 for i in range(3) ]
        result[0] = vcopy[0] / vector_length
        result[1] = vcopy[1] / vector_length
        result[2] = vcopy[2] / vector_length
        return result
    
    def vector_subtraction_3D( self, v1, v2 ):
        v1copy = copy.deepcopy( v1 )
        v2copy = copy.deepcopy( v2 )

        result = [ 0.0 for i in range(3) ]
        result[0] = v1copy[0] - v2copy[0] 
        result[1] = v1copy[1] - v2copy[1]
        result[2] = v1copy[2] - v2copy[2]
        return result
    
    def dot_product( self, v1, v2 ):
        v1copy = copy.deepcopy( v1 )
        v2copy = copy.deepcopy( v2 )
        return  v1copy[0]*v2copy[0] + v1copy[1]*v2copy[1] + v1copy[2]*v2copy[2] 
    
    def cross_product_3D( self, v1, v2 ):
        v1copy = copy.deepcopy( v1 )
        v2copy = copy.deepcopy( v2 )
        result = [ 0.0 for i in range(3) ]
        result[0] = v1copy[1]*v2copy[2] - v1copy[2]*v2copy[1]
        result[1] = v1copy[2]*v2copy[0] - v1copy[0]*v2copy[2]
        result[2] = v1copy[0]*v2copy[1] - v1copy[1]*v2copy[0]
        return result
    
    def angle_between_two_vectors( self, v1, v2 ):
        v1_unit = self.unit_vector_3D( copy.deepcopy( v1 ) )
        v2_unit = self.unit_vector_3D( copy.deepcopy( v2 ))
        dot     = self.dot_product( v1_unit, v2_unit )
        return np.arccos( dot )
    
    def rad2deg( self, rad ):
        return rad * ( 180/np.pi )
    
    def deg2rad( self, deg ):
        return deg * ( np.pi/180 )


import json
import scipy.sparse
import scipy.sparse.linalg
import numpy

def solveTruss(Truss):

    #with open('truss.json','rb') as file: Truss = json.load(file)
    
    num_joints = len(Truss['Joints'])
    dimension = 3
    
    A = scipy.sparse.lil_matrix((num_joints*dimension, num_joints*dimension))
    
    for Beam in Truss['Beams']:
        for dim_T in range(dimension):
            for dim_u in range(dimension):
                s = Beam['SpringConst'] * Beam['ConnectionVector'][dim_T] * Beam['ConnectionVector'][dim_u]
                id_i = Beam['JointIndex_i']
                id_j = Beam['JointIndex_j']
                
                A[id_i*dimension+dim_T, id_i*dimension+dim_u] += s
                A[id_i*dimension+dim_T, id_j*dimension+dim_u] -= s
                A[id_j*dimension+dim_T, id_i*dimension+dim_u] -= s
                A[id_j*dimension+dim_T, id_j*dimension+dim_u] += s
    
    
    
    f = numpy.array([force_component for joint in Truss['Joints'] for force_component in joint['Force']])
    
    
    for fixedJoint in Truss['FixedJoints']:
        for dim_T in range(dimension):
            for col_id in range(num_joints*dimension):
                A[fixedJoint*dimension+dim_T, col_id] = 0
            
            A[fixedJoint*dimension+dim_T, fixedJoint*dimension+dim_T] = 1
            f[fixedJoint*dimension+dim_T] = 0
    
    
    A = scipy.sparse.csc_matrix(A)
    u = scipy.sparse.linalg.spsolve(A,f)
    Displacements = u.reshape((num_joints, dimension))
    
    for i, Displacement in enumerate(Displacements):
        Truss['Joints'][i]['Displacement'] = Displacement.tolist()
        Truss['Joints'][i]['DisplacedPosition'] = (Displacement + numpy.array(Truss['Joints'][i]['Position'])).tolist()
    
    max_tension_magnitude = 0
    
    for i, Beam in enumerate(Truss['Beams']):
        id_i = Beam['JointIndex_i']
        id_j = Beam['JointIndex_j']
        Truss['Beams'][i]['Tension'] =  Beam['SpringConst'] * numpy.dot(numpy.array(Beam['ConnectionVector']) , (numpy.array(Truss['Joints'][id_j]['Displacement']) - numpy.array(Truss['Joints'][id_i]['Displacement'])))
        max_tension_magnitude = max(abs(Truss['Beams'][i]['Tension']), max_tension_magnitude)
        
        print Truss['Beams'][i]['Tension']
    
    if max_tension_magnitude == 0: max_tension_magnitude = 1e-10
    
    for i, Beam in enumerate(Truss['Beams']):
        Truss['Beams'][i]['TensionRelativeMagnitude'] = abs(Truss['Beams'][i]['Tension']) / max_tension_magnitude
    
    #with open('truss.json','wb') as file: json.dump(Truss, file)
    return Truss
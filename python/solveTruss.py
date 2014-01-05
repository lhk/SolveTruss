import scipy.sparse
import scipy.sparse.linalg
import numpy

def solveTruss(Truss):
    num_joints = len(Truss['Joints'])
    num_beams = len(Truss['Beams'])
    dimension = 3

    # our system of equations: Au=f    
    A = scipy.sparse.lil_matrix((num_joints*dimension, num_joints*dimension))
    f = numpy.array([force_component for joint in Truss['Joints'] for force_component in joint['ExternalForce']])
    
    
    # put beam-wise data in the equation system
    for Beam in Truss['Beams']:
        sprint_const = Beam['ElasticModulus'] * Beam['CrossSectionArea'] / Beam['Length']
        id_i = Beam['JointIndex_i']
        id_j = Beam['JointIndex_j']        
        
        # Add weight of the beams
        beam_mass = Beam['CrossSectionArea'] * Beam['Length'] * Beam['Density']        
        f[id_i*dimension+1] -= beam_mass/2 * 9.81
        f[id_j*dimension+1] -= beam_mass/2 * 9.81        
        
        for dim_T in range(dimension):
            for dim_u in range(dimension):
                s = sprint_const * Beam['ConnectionVector'][dim_T] * Beam['ConnectionVector'][dim_u]
                A[id_i*dimension+dim_T, id_i*dimension+dim_u] += s
                A[id_i*dimension+dim_T, id_j*dimension+dim_u] -= s
                A[id_j*dimension+dim_T, id_i*dimension+dim_u] -= s
                A[id_j*dimension+dim_T, id_j*dimension+dim_u] += s
                
    # overwrite the equations for the pin supports: (dx, dy, dz) = 0
    for fixedJoint in Truss['FixedJoints']:
        for dim_T in range(dimension):
            for col_id in range(num_joints*dimension):
                A[fixedJoint*dimension+dim_T, col_id] = 0
            
            A[fixedJoint*dimension+dim_T, fixedJoint*dimension+dim_T] = 1
            f[fixedJoint*dimension+dim_T] = 0
    
    # solve the equation system
    A = scipy.sparse.csc_matrix(A)
    u = scipy.sparse.linalg.spsolve(A,f)

    # move the result into our datastructure    
    Displacements = u.reshape((num_joints, dimension))    
    for i, Displacement in enumerate(Displacements):
        Truss['Joints'][i]['Displacement'] = Displacement.tolist()
        Truss['Joints'][i]['DisplacedPosition'] = (Displacement + numpy.array(Truss['Joints'][i]['Position'])).tolist()
    
    
    # Calculate the tension and stress in every beam.
    # Calculate material cost.
    # Calculate a simple efficiency score.
    # Find the largest absolute tension.
    # Check if the truss is not so distorted that it makes the linear approx inaccurate.
    # Check if the maximal stress of a beam is exceeded.
    max_tension_magnitude = 0
    Truss['BeamOrientationTest'] = True
    Truss['BeamStressLimitTest'] = True
    Truss['MaterialCost'] = 0
    Truss['EfficiencyScore'] = 0
    for i, Beam in enumerate(Truss['Beams']):
        id_i = Beam['JointIndex_i']
        id_j = Beam['JointIndex_j']
        sprint_const = Beam['ElasticModulus'] * Beam['CrossSectionArea'] / Beam['Length']        
        Displacement = numpy.array(Truss['Joints'][id_j]['Displacement']) - numpy.array(Truss['Joints'][id_i]['Displacement'])
        BeamDirection = numpy.array(Beam['ConnectionVector'])        
        ParallelDisplacement = numpy.dot(BeamDirection, Displacement)        
        OrthogonalDisplacement = Displacement-(BeamDirection*ParallelDisplacement)
        
        Truss['Beams'][i]['Tension'] = sprint_const * ParallelDisplacement
        max_tension_magnitude = max(abs(Truss['Beams'][i]['Tension']), max_tension_magnitude)
        Truss['Beams'][i]['Stress'] = Truss['Beams'][i]['Tension'] / Truss['Beams'][i]['CrossSectionArea']        
        
        # if the bar is less than 6 degrees (atan(1/10)) disoriented the approximation should be okay
        if Beam['Length'] < 10 * numpy.linalg.norm(OrthogonalDisplacement):
            Truss['BeamOrientationTest'] = False
            
        if abs(Truss['Beams'][i]['Stress']) > Truss['Beams'][i]['StressLimit']:
            Truss['BeamStressLimitTest'] = False
        
        Truss['MaterialCost'] += Beam['CostPerMass'] * Beam['Length'] * Beam['CrossSectionArea'] * Beam['Density']
        Truss['EfficiencyScore'] += abs(Truss['Beams'][i]['Stress']) / Truss['Beams'][i]['StressLimit']
        
    Truss['EfficiencyScore'] /= num_beams
    
    # to avoid division by zero
    if max_tension_magnitude == 0: max_tension_magnitude = 1e-10
    
    # calculate the relative tension, needed for visualization
    for i, Beam in enumerate(Truss['Beams']):
        Truss['Beams'][i]['TensionRelativeMagnitude'] = abs(Truss['Beams'][i]['Tension']) / max_tension_magnitude
    
    return Truss
import scipy.sparse
import scipy.sparse.linalg
import numpy

def solveTruss(Truss):
    num_joints = len(Truss['Joints'])
    num_beams = len(Truss['Beams'])
    dimension = 3

    # our system of equations: Au=f    
    A = {}
    weight_f = numpy.array([0]*(num_joints*dimension))
    
    # put beam-wise data in the equation system
    for Beam in Truss['Beams']:
        sprint_const = Beam['ElasticModulus'] * Beam['CrossSectionArea'] / Beam['Length']
        id_i = Beam['JointIndex_i']
        id_j = Beam['JointIndex_j']        
        
        # Add weight of the beams
        beam_mass = Beam['CrossSectionArea'] * Beam['Length'] * Beam['Density']        
        weight_f[id_i*dimension+1] -= beam_mass/2 * 9.81
        weight_f[id_j*dimension+1] -= beam_mass/2 * 9.81        
        
        # Core algorithm: Connects the beams to the equilibrium equations.
        for dim_T in range(dimension):
            for dim_u in range(dimension):
                s = sprint_const * Beam['ConnectionVector'][dim_T] * Beam['ConnectionVector'][dim_u]
                                
                ii = str(id_i*dimension+dim_T) + ","  + str(id_i*dimension+dim_u)
                ij = str(id_i*dimension+dim_T) + ","  + str(id_j*dimension+dim_u)
                ji = str(id_j*dimension+dim_T) + ","  + str(id_i*dimension+dim_u)
                jj = str(id_j*dimension+dim_T) + ","  + str(id_j*dimension+dim_u)
                
                if not ii in A: A[ii] = [id_i*dimension+dim_T, id_i*dimension+dim_u, 0]
                if not ij in A: A[ij] = [id_i*dimension+dim_T, id_j*dimension+dim_u, 0]
                if not ji in A: A[ji] = [id_j*dimension+dim_T, id_i*dimension+dim_u, 0]
                if not jj in A: A[jj] = [id_j*dimension+dim_T, id_j*dimension+dim_u, 0]                    
                    
                A[ii][2] += s
                A[ij][2] -= s
                A[ji][2] -= s
                A[jj][2] += s
                

    # Convert the representation of the equation matrix, necessary for performance reasons
    IJV = A.values()
    row = [ijv[0] for ijv in IJV]
    col = [ijv[1] for ijv in IJV]
    data = [ijv[2] for ijv in IJV]    
    A = scipy.sparse.coo_matrix((data,(row,col)), shape=(num_joints*dimension, num_joints*dimension))    
    A = scipy.sparse.csc_matrix(A)
        
    # overwrite the equations for the pin supports: (dx, dy, dz) = 0
    for fixedJoint in Truss['FixedJoints']:
        for dim_T in range(dimension):
            for col_id in range(num_joints*dimension):
                i = str(fixedJoint*dimension+dim_T) + ","  + str(col_id)
                if i in A: A[i][2] = 0
            
            i = str(fixedJoint*dimension+dim_T) + ","  + str(fixedJoint*dimension+dim_T)
            if i in A: A[i][2] = 1

    for LoadScenariosID in len(Truss['LoadScenarios']):

        num_free_joints = num_joints - len(Truss['FixedJoints'])
        f = weight_f + numpy.array([[Truss['LoadScenarios'][LoadScenariosID]['Force'][dim] / num_free_joints for dim in range(dimension) ] for i in range(num_joints)])
        
        # overwrite the equations for the pin supports: (dx, dy, dz) = 0
        for fixedJoint in Truss['FixedJoints']:
                f[fixedJoint*dimension+dim_T] = 0
        
        
        
        # solve the equation system    
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
        Truss['LoadScenarios'][LoadScenariosID]['BeamOrientationTest'] = True
        Truss['LoadScenarios'][LoadScenariosID]['BeamStressLimitTest'] = True
        Truss['LoadScenarios'][LoadScenariosID]['MaterialCost'] = 0
        Truss['LoadScenarios'][LoadScenariosID]['EfficiencyScore'] = 0
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
                Truss['LoadScenarios'][LoadScenariosID]['BeamOrientationTest'] = False
                
            if abs(Truss['Beams'][i]['Stress']) > Truss['Beams'][i]['StressLimit']:
                Truss['LoadScenarios'][LoadScenariosID]['BeamStressLimitTest'] = False
            
            Truss['LoadScenarios'][LoadScenariosID]['MaterialCost'] += Beam['CostPerMass'] * Beam['Length'] * Beam['CrossSectionArea'] * Beam['Density']
            Truss['LoadScenarios'][LoadScenariosID]['EfficiencyScore'] += abs(Truss['Beams'][i]['Stress']) / Truss['Beams'][i]['StressLimit']
            
        Truss['LoadScenarios'][LoadScenariosID]['EfficiencyScore'] /= num_beams
        
        # to avoid division by zero
        if max_tension_magnitude == 0: max_tension_magnitude = 1e-10
        
        # calculate the relative tension, needed for visualization
        for i, Beam in enumerate(Truss['Beams']):
            Truss['Beams'][i]['TensionRelativeMagnitude'] = abs(Truss['Beams'][i]['Tension']) / max_tension_magnitude
        
        Truss['LoadScenarios'][LoadScenariosID]['Score'] = Truss['LoadScenarios'][LoadScenariosID]['MaterialCost'] * (1-Truss['LoadScenarios'][LoadScenariosID]['EfficiencyScore'])
    
    
    return Truss
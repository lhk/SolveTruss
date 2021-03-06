import math
import random
import json
import zlib

def generateTruss():
    random.seed()
    sections = 6
    Truss={}

    # Exampe: Bridge

    # Generate all nodes    
    Truss['Joints'] = [None]*(4*sections)
    for i in range(sections):
        
        x = 2.0 * i / (sections-1) - 1
        
        lower = 0
        upper = 1
        
        x *= 15
    
        scale = 15
        Truss['Joints'][4*i+0] = {'Position': [scale*x, scale*lower,     0]};
        Truss['Joints'][4*i+1] = {'Position': [scale*x, scale*upper,     0]};
        Truss['Joints'][4*i+2] = {'Position': [scale*x, scale*lower, scale]};
        Truss['Joints'][4*i+3] = {'Position': [scale*x, scale*upper, scale]};
        
    
    # Iterate over all pairs of nodes and select connecting beams
    Truss['Beams'] = []
    for i in range(4*sections-1):
        for j in range(i+1, 4*sections):
    
            connection_vector = [Truss['Joints'][j]['Position'][k]-Truss['Joints'][i]['Position'][k] for k in range(3)]
            length = math.sqrt(sum([connection_vector[k] * connection_vector[k] for k in range(3)]))
            for k in range(3): connection_vector[k] /= length
            
            beam = {
                    'JointIndex_i': i, 
                    'JointIndex_j': j, 
                    'ConnectionVector': connection_vector, 
                    'Length': length,
                    'ElasticModulus': 210e9, # ElasticModulus of steel in Pascal (SI-MKS)
                    'Density': 7.8e3,        # Density of steel in kg m^-3 (SI-MKS)
                    'StressLimit': 250e6,    # StressLimit of steel in Pascal (SI-MKS)
                    'CrossSectionArea': 0.8, # CrossSectionArea of the beam in m^2 (SI-MKS)
                    'CostPerMass': 0.7       # dollar per kg
                    }
            
            i_x = i//4
            i_y = i % 2
            i_z = (i//2) % 2
            
            j_x = j//4        
            j_y = j % 2
            j_z = (j//2) % 2
    
            if i_x == j_x:
                if i_y == j_y:
                    beam['CrossSectionArea'] = 0.1
                    Truss['Beams'].append(beam)
                    
                if i_z == j_z:
                    Truss['Beams'].append(beam)
            
            if j_x - i_x == 1:
                if i_z == j_z and i_y == j_y:
                    Truss['Beams'].append(beam)
                    
                if i_z == j_z and i_y != j_y:# and (i_x+1 > sections/2) == (i_y < j_y):
                    Truss['Beams'].append(beam)
                    
                if i_z != j_z and i_y == j_y:
                    beam['CrossSectionArea'] = 0.1
                    Truss['Beams'].append(beam)
            
    # Define pin supports
    Truss['FixedJoints'] = [0, 1, 2, 3, 4*sections-4, 4*sections-3, 4*sections-2, 4*sections-1]
    
    Truss['LoadScenarios'] = []
    Truss['LoadScenarios'].append({'Force' :[1e5, -2e7, 1e6]})
    Truss['LoadScenarios'].append({'Force' :[1e5, -2e7, -1e6]})
    Truss['LoadScenarios'].append({'Force' :[-1e5, -2e7, 1e6]})
    Truss['LoadScenarios'].append({'Force' :[-1e5, -2e7, -1e6]})
    
    return Truss
    
if __name__ == "__main__":
    with open('pool/truss.z', 'wb') as file: 
        file.write(zlib.compress(json.dumps(generateTruss())))
#import json
import math
import random

def generateTurss(sections = 15):
    random.seed()
    Truss={}
    
    Truss['Joints'] = [None]*(4*sections)
    
    for i in range(sections):
        
        x = 2.0 * i / (sections-1) - 1
        
        lower = -1*(x+1)*(x-1)
        upper = -0.5*(x+1)*(x-1)+0.7
        
        x *= 3

        rand_range = 50        
        
        Truss['Joints'][4*i+0] = {'Position': [x, lower, 0], 'Force': [0, -100 +random.uniform(-rand_range,rand_range), random.uniform(-rand_range,rand_range)]};
        Truss['Joints'][4*i+1] = {'Position': [x, upper, 0], 'Force': [0, -100 +random.uniform(-rand_range,rand_range), random.uniform(-rand_range,rand_range)]};
        Truss['Joints'][4*i+2] = {'Position': [x, lower, 1], 'Force': [0, -100 +random.uniform(-rand_range,rand_range), random.uniform(-rand_range,rand_range)]};
        Truss['Joints'][4*i+3] = {'Position': [x, upper, 1], 'Force': [0, -100 +random.uniform(-rand_range,rand_range), random.uniform(-rand_range,rand_range)]};
        
    
    
    Truss['Beams'] = []
    for i in range(4*sections-1):
        for j in range(i+1, 4*sections):
    
            connection_vector = [Truss['Joints'][j]['Position'][k]-Truss['Joints'][i]['Position'][k] for k in range(3)]
            length = math.sqrt(sum([connection_vector[k] * connection_vector[k] for k in range(3)]))
            for k in range(3): connection_vector[k] /= length
            
            beam = {'JointIndex_i': i, 'JointIndex_j': j, 'ConnectionVector': connection_vector, 'Length': length}
            
            i_x = i//4
            i_y = i % 2
            i_z = (i//2) % 2
            
            j_x = j//4        
            j_y = j % 2
            j_z = (j//2) % 2
    
            if i_x == j_x:
                if i_y == j_y:
                    beam['SpringConst'] = length * 1e7
                    Truss['Beams'].append(beam)
                    
                if i_z == j_z:
                    beam['SpringConst'] = length * 1e7
                    Truss['Beams'].append(beam)
            
            if j_x - i_x == 1:
                if i_z == j_z and i_y == j_y:
                    beam['SpringConst'] = length * 1e7
                    Truss['Beams'].append(beam)
                    
                if i_z == j_z and i_y != j_y and (i_x+1 > sections/2) != (i_y < j_y):
                    beam['SpringConst'] = length * 1e7
                    Truss['Beams'].append(beam)
                    
                if i_z != j_z and i_y == j_y:
                    beam['SpringConst'] = length * 1e7
                    Truss['Beams'].append(beam)
                
            
    
    Truss['FixedJoints'] = [0, 1, 2, 3, 4*sections-4, 4*sections-3, 4*sections-2, 4*sections-1]
    
    #with open('truss.json','wb') as file: json.dump(Truss, file)
    return Truss
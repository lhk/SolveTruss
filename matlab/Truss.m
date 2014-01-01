classdef Truss < handle
    %TRUSS Representation of a 3D Truss
    %   TODO: Detailed explanation goes here
    
    properties
        % 3 by n matrix, for the points of joints of the truss.
        % columns = (position_x, position_y, position_z, force_x, force_y, force_z)
        Joints
        
        % 3 by n matrix, for the external forces on the joints.
        % columns = (force_x, force_y, force_z)
        Forces
        
        % 3 by m index-matrix that selects pairs of joints connected by beams.
        % Also holds the spring coefficient (from Hooke's Law) for each
        % beam.
        % columns = (joint_id_i, joint_id_j, spring_coefficient)
        Beams
        
        % 4 by k matrix that specifies which joints are restricted to move
        % in the given direction. 
        % columns = (joint_id, direction_x, direction_y, direction_z)
        Supports
        
        
        %%% The properties below will be filled by the solve() method.
        
        % 1 by m vector, for the tension forces in the beams
        BeamTensions
        
        % 3 by n matrix, for the displacements of the joints.
        Displacements
        
        % 3 by n matrix, for the displaced joints, this is just the sum of
        % Joints + Displacements.
        DisplacedJoints
    end
    
    methods
        function this=Truss()
            % Create example Truss (Syndey Harbour Bridge)
            sections = 7;
            this.Joints = zeros(3, 4*sections);
            
            % Create example joints
            for i=1:sections
                x = 2*(i-1)/(sections-1) - 1;
                
                lower_z = -1*(x+1)*(x-1)+0;
                upper_z = -0.5*(x+1)*(x-1)+0.7; 
                
                x = x * 3;
                
                % Ugh, 1-based indexing sucks.
                this.Joints(:,4*i-3) = [x 0 lower_z]';
                this.Joints(:,4*i-2) = [x 0 upper_z]';
                this.Joints(:,4*i-1) = [x 1 lower_z]';
                this.Joints(:,4*i-0) = [x 1 upper_z]';
                
                this.Forces(:,4*i-3) = [0 0 -100]';
                this.Forces(:,4*i-2) = [0 0 -100]';
                this.Forces(:,4*i-1) = [0 0 -100]';
                this.Forces(:,4*i-0) = [0 0 -100]';
            end
            
            % Create example beams
            % Look at all pairs and check if they're connected
            this.Beams = [];
            for i=1:(4*sections-1)
                for j=(i+1):(4*sections)
                    
                    dist = norm(this.Joints(1:3,j)-this.Joints(1:3,i));
                    
                    i_section = floor((i-1)/4);
                    i_y = floor((i-(i_section-1)*4-1)/2);
                    i_z = mod((i-(i_section-1)*4)-1,2);
                    
                    j_section = floor((j-1)/4);
                    j_y = floor((j-(j_section-1)*4-1)/2);
                    j_z = mod((j-(j_section-1)*4)-1,2);
                    
                    
                    
                    if(i_section == j_section && (i_y==j_y || i_z==j_z))
                        this.Beams = [this.Beams [i j dist*1e7]'];
                    end
                        
                    if(abs(i_section - j_section)==1)
                        if(i_y==j_y)
                            if( size(this.Joints,2)/8 > i_section+1)
                                if(i_z < j_z)
                                    this.Beams = [this.Beams [i j dist*1e3]'];
                                end
                            end
                            if( size(this.Joints,2)/8 <= i_section+1)
                                if(i_z > j_z)
                                    this.Beams = [this.Beams [i j dist*1e3]'];
                                end
                            end
                        end
                        if(i_z==j_z)
                            this.Beams = [this.Beams [i j dist*1e7]'];
                        end
                    end
                end
            end
            
            % Create Supports
            this.Supports = [];
            
            this.Supports = [this.Supports [1 1 0 0]'];
            this.Supports = [this.Supports [1 0 1 0]'];
            this.Supports = [this.Supports [1 0 0 1]'];
            
            this.Supports = [this.Supports [2 1 0 0]'];
            this.Supports = [this.Supports [2 0 1 0]'];
            this.Supports = [this.Supports [2 0 0 1]'];
            
            this.Supports = [this.Supports [3 1 0 0]'];
            this.Supports = [this.Supports [3 0 1 0]'];
            this.Supports = [this.Supports [3 0 0 1]'];
            
            this.Supports = [this.Supports [4 1 0 0]'];
            this.Supports = [this.Supports [4 0 1 0]'];
            this.Supports = [this.Supports [4 0 0 1]'];
            
            
            
            this.Supports = [this.Supports [sections*4 1 0 0]'];
            this.Supports = [this.Supports [sections*4 0 1 0]'];
            this.Supports = [this.Supports [sections*4 0 0 1]'];
            
            this.Supports = [this.Supports [sections*4-1 1 0 0]'];
            this.Supports = [this.Supports [sections*4-1 0 1 0]'];
            this.Supports = [this.Supports [sections*4-1 0 0 1]'];
            
            this.Supports = [this.Supports [sections*4-2 1 0 0]'];
            this.Supports = [this.Supports [sections*4-2 0 1 0]'];
            this.Supports = [this.Supports [sections*4-2 0 0 1]'];
            
            this.Supports = [this.Supports [sections*4-3 1 0 0]'];
            this.Supports = [this.Supports [sections*4-3 0 1 0]'];
            this.Supports = [this.Supports [sections*4-3 0 0 1]'];
        end
        
        
        function plot(this)
            h = plot3(this.Joints(1,:),this.Joints(2,:),this.Joints(3,:), 'bo');
            hold on
            
            cmap = jet(101);
            
            c = abs(this.BeamTensions');
            c = (c-min(c)) / (max(c)-min(c));
            c = cmap(int32(ceil(c*100))+1,:);
            
            set(0,'DefaultAxesColorOrder',c);
            plot3( ...
                [this.DisplacedJoints(1,this.Beams(1,:));this.DisplacedJoints(1,this.Beams(2,:))], ...
                [this.DisplacedJoints(2,this.Beams(1,:));this.DisplacedJoints(2,this.Beams(2,:))], ...
                [this.DisplacedJoints(3,this.Beams(1,:));this.DisplacedJoints(3,this.Beams(2,:))], ...
                'LineWidth',3);
            
            hold off
            
            % Settings to make the plot window useable.
            camproj('perspective');
            axis vis3d;
            axis equal;
            cameratoolbar('Show');
            cameratoolbar('SetMode','orbit');
            camva(70);
            campos([3 3 3]);
            axis off
        end
        
        
        function solve(this)
            % Generate sparse solution matrix A*u=f
            dimension = 3;
            num_joints = size(this.Joints,2);
            num_beams = size(this.Beams,2);
            num_supports = size(this.Supports,2);
            
            % One equation for each joint and dimension.
            A = spalloc(num_joints*dimension, num_joints*dimension, (num_joints+5)*dimension*dimension);
            
            
            % Add beam coefficients
            for beam_id=1:num_beams
                
                joint_id_i = this.Beams(1,beam_id);
                joint_id_j = this.Beams(2,beam_id);
                spring_coefficient = this.Beams(3,beam_id);
                
                lambda_ij = this.Joints(1:dimension, joint_id_j) - this.Joints(1:dimension, joint_id_i);
                lambda_ij = lambda_ij / norm(lambda_ij);
                

                for dim_T=1:dimension
                    for dim_u=1:dimension
                        A((joint_id_i-1)*dimension+dim_T,(joint_id_i-1)*dimension+dim_u) = A((joint_id_i-1)*dimension+dim_T,(joint_id_i-1)*dimension+dim_u) + spring_coefficient * lambda_ij(dim_T) * lambda_ij(dim_u);
                        A((joint_id_i-1)*dimension+dim_T,(joint_id_j-1)*dimension+dim_u) = A((joint_id_i-1)*dimension+dim_T,(joint_id_j-1)*dimension+dim_u) - spring_coefficient * lambda_ij(dim_T) * lambda_ij(dim_u);
                        
                        A((joint_id_j-1)*dimension+dim_T,(joint_id_i-1)*dimension+dim_u) = A((joint_id_j-1)*dimension+dim_T,(joint_id_i-1)*dimension+dim_u) - spring_coefficient * lambda_ij(dim_T) * lambda_ij(dim_u);
                        A((joint_id_j-1)*dimension+dim_T,(joint_id_j-1)*dimension+dim_u) = A((joint_id_j-1)*dimension+dim_T,(joint_id_j-1)*dimension+dim_u) + spring_coefficient * lambda_ij(dim_T) * lambda_ij(dim_u);

                    end
                end
            end
            
            % Create right hand side (external forces)
            f = reshape(this.Forces, num_joints * dimension,1);
            
            % Write Support constraints
            this.Supports = sortrows(this.Supports')';
            
            dim = 1;
            prev_joint_id = 0;
            
            for support_id=1:num_supports
                joint_id = this.Supports(1,support_id);
                if(prev_joint_id == joint_id)
                   dim = dim+1;                   
                else
                    dim = 1;
                end
                
                % Clear the row
                A((joint_id-1)*dimension+dim,:) = 0;
                f((joint_id-1)*dimension+dim) = 0;
                
                
                A((joint_id-1)*dimension+dim,(joint_id-1)*dimension+1:(joint_id-1)*dimension+dimension) = this.Supports(2:dimension+1,support_id)';
                
                
                prev_joint_id = joint_id;
            end

            % Solve
            u=A\f;
            
            % Save solution
            this.Displacements = reshape(u, dimension, num_joints);
            this.DisplacedJoints = this.Joints(1:dimension,:) + this.Displacements;
            
            % Calculate tension forces in the beams.
            this.BeamTensions = zeros(1, num_beams);
            for beam_id=1:num_beams
                
                joint_id_i = this.Beams(1,beam_id);
                joint_id_j = this.Beams(2,beam_id);
                spring_coefficient = this.Beams(3,beam_id);
                
                lambda_ij = this.Joints(1:dimension, joint_id_j) - this.Joints(1:dimension, joint_id_i);
                lambda_ij = lambda_ij / norm(lambda_ij);
                
                delta_length = dot(lambda_ij,(this.Displacements(:,joint_id_j)-this.Displacements(:,joint_id_i)));
                
                this.BeamTensions(beam_id) = delta_length * spring_coefficient;
            end
        end        
    end    
end


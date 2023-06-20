function [] = ceilPlane2(pc, varargin)

p = inputParser;
addParameter(p, 'maxDistance', 0.2, @isnumeric);
addParameter(p, 'percentage', 0.3, @isnumeric);
addParameter(p, 'refVector', [0 0 1]);
parse(p, varargin{:});
maxDistance = p.Results.maxDistance;
percentage = p.Results.percentage;
refVector = p.Results.refVector;


[~,idx] = sort(pc.Location(:,3));
% get the top 30% indices
top30 = idx(end-floor(percentage*length(idx)):end);
% fit a plane to the top 30% points
top30PC = select(pc, top30);
top30Z = top30PC.Location(:,3);
top30Y = top30PC.Location(:,2);
top30X = top30PC.Location(:,1);

top30_zmin = min(top30PC.Location(:,3));
top30_zmax = max(top30PC.Location(:,3));

step_size = 0.001;

z_grid =  top30_zmin:step_size:top30_zmax;
pdf_z = zeros(1,length(z_grid)-1);
pdf_x = zeros(1,length(z_grid)-1);
pdf_y = zeros(1,length(z_grid)-1);
for i= 1:length(z_grid)-1
    
    top30_low_end = top30Z > z_grid(i);
    top30_high_end = top30Z <= z_grid(i+1);
    
    pdf_z(i) = length(top30Z(top30_low_end & top30_high_end));
    
    pdf_x(i) = std(top30X(top30_low_end & top30_high_end));
    pdf_y(i) = std(top30Y(top30_low_end & top30_high_end));
    
end
figure
hold on
plot(z_grid(1:end-1),pdf_z/max(pdf_z) )
% plot(z_grid(1:end-1),pdf_x )
% plot(z_grid(1:end-1),pdf_y )
plot(z_grid(1:end-1),(pdf_x+pdf_y)/2 )

% pdf_z_local_max = islocalmax(pdf);

% zgrid_ = z_grid(1:end-1)



% plot(zgrid_,pdf,zgrid_(pdf_z_local_max),pdf(pdf_z_local_max),'r*')
% [~,idx] = max(pdf);
end
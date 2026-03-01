function h = draw_quadcopter(position, orientation, scale)
if nargin < 3, scale = 1; end
if nargin < 2, orientation = [0, 0, 0]; end
arm_length = 0.8 * scale; body_radius = 0.3 * scale; prop_radius = 0.4 * scale;
roll = orientation(1); pitch = orientation(2); yaw = orientation(3);
Rx = [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)];
Ry = [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)];
Rz = [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];
R = Rz * Ry * Rx;
arm1 = R*[0,arm_length;0,0;0,0] + position;
arm2 = R*[0,-arm_length;0,0;0,0] + position;
arm3 = R*[0,0;0,arm_length;0,0] + position;
arm4 = R*[0,0;0,-arm_length;0,0] + position;
hold on;
h.arm1 = plot3(arm1(1,:),arm1(2,:),arm1(3,:),'r-','LineWidth',4);
h.arm2 = plot3(arm2(1,:),arm2(2,:),arm2(3,:),'r-','LineWidth',4);
h.arm3 = plot3(arm3(1,:),arm3(2,:),arm3(3,:),'b-','LineWidth',4);
h.arm4 = plot3(arm4(1,:),arm4(2,:),arm4(3,:),'b-','LineWidth',4);
[X,Y,Z] = sphere(20);
h.body = surf(X*body_radius+position(1), Y*body_radius+position(2), Z*body_radius+position(3), 'FaceColor',[0.2,0.2,0.2],'EdgeColor','none','FaceAlpha',0.8);
prop_pos = [arm1(:,2),arm2(:,2),arm3(:,2),arm4(:,2)];
for i=1:4
  [X,Y,Z]=cylinder(prop_radius,30); Z=Z*0.02-0.01;
  for j=1:size(X,1), for k=1:size(X,2)
    pt=R*[X(j,k);Y(j,k);Z(j,k)];
    X(j,k)=pt(1)+prop_pos(1,i); Y(j,k)=pt(2)+prop_pos(2,i); Z(j,k)=pt(3)+prop_pos(3,i);
  end, end
  h.props(i)=surf(X,Y,Z,'FaceColor',[0.3,0.3,0.3],'EdgeColor','none','FaceAlpha',0.6);
end
arrow_end = R*[arm_length*1.2;0;0] + position;
h.arrow = plot3([position(1),arrow_end(1)],[position(2),arrow_end(2)],[position(3),arrow_end(3)],'g-','LineWidth',3);
h.arrow_head = plot3(arrow_end(1),arrow_end(2),arrow_end(3),'g^','MarkerSize',12,'MarkerFaceColor','g');
end

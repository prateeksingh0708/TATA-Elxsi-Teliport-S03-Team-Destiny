function update_quadcopter(h, position, orientation)
if nargin<3, orientation=[0,0,0]; end
scale=1; arm_length=0.8*scale; body_radius=0.3*scale; prop_radius=0.4*scale;
roll=orientation(1); pitch=orientation(2); yaw=orientation(3);
Rx=[1,0,0;0,cos(roll),-sin(roll);0,sin(roll),cos(roll)];
Ry=[cos(pitch),0,sin(pitch);0,1,0;-sin(pitch),0,cos(pitch)];
Rz=[cos(yaw),-sin(yaw),0;sin(yaw),cos(yaw),0;0,0,1];
R=Rz*Ry*Rx;
arm1=R*[0,arm_length;0,0;0,0]+position;
arm2=R*[0,-arm_length;0,0;0,0]+position;
arm3=R*[0,0;0,arm_length;0,0]+position;
arm4=R*[0,0;0,-arm_length;0,0]+position;
set(h.arm1,'XData',arm1(1,:),'YData',arm1(2,:),'ZData',arm1(3,:));
set(h.arm2,'XData',arm2(1,:),'YData',arm2(2,:),'ZData',arm2(3,:));
set(h.arm3,'XData',arm3(1,:),'YData',arm3(2,:),'ZData',arm3(3,:));
set(h.arm4,'XData',arm4(1,:),'YData',arm4(2,:),'ZData',arm4(3,:));
[X,Y,Z]=sphere(20);
set(h.body,'XData',X*body_radius+position(1),'YData',Y*body_radius+position(2),'ZData',Z*body_radius+position(3));
prop_pos=[arm1(:,2),arm2(:,2),arm3(:,2),arm4(:,2)];
for i=1:4
  [X,Y,Z]=cylinder(prop_radius,30); Z=Z*0.02-0.01;
  for j=1:size(X,1), for k=1:size(X,2)
    pt=R*[X(j,k);Y(j,k);Z(j,k)];
    X(j,k)=pt(1)+prop_pos(1,i); Y(j,k)=pt(2)+prop_pos(2,i); Z(j,k)=pt(3)+prop_pos(3,i);
  end, end
  set(h.props(i),'XData',X,'YData',Y,'ZData',Z);
end
arrow_end=R*[arm_length*1.2;0;0]+position;
set(h.arrow,'XData',[position(1),arrow_end(1)],'YData',[position(2),arrow_end(2)],'ZData',[position(3),arrow_end(3)]);
set(h.arrow_head,'XData',arrow_end(1),'YData',arrow_end(2),'ZData',arrow_end(3));
drawnow;
end

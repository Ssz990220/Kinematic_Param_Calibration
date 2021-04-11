close all
p1 = [0,0,0];
p2 = [80,0,0];
T0 = eye(3);
T1 = @(x,y) roty(y)*rotx(x);


y = -10:0.05:10;
x = -10:0.05:10;

filename = 'testAnimated.gif';
fig = figure
for i = 1:size(y,2)  
    view_holes([T0,p1';zeros(1,3),1],100,false);
    hold on
    B = T1(x(i),y(i));
    view_holes([B,p2';zeros(1,3),1],100,false);
    [p_c1,p_c2,l]=solve_common_normal([0,0,1],p1, B(1:3,3)', p2);
    plot3([p_c1(1),p_c2(1)],[p_c1(2),p_c2(2)],[p_c1(3),p_c2(3)])
    plot3([p1(1),p_c1(1)],[p1(2),p_c1(2)],[p1(3),p_c1(3)])
    plot3([p2(1),p_c2(1)],[p2(2),p_c2(2)],[p2(3),p_c2(3)])
    view([30,60])   
    axis equal
    drawnow
    hold off
    frame = getframe(fig); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if i == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.016); 
      else 
          imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.016); 
      end
end

